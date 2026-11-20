#define IMX519_TRACE(...) do { if(getenv("IMX519_DEBUG")) LOGE(__VA_ARGS__); } while(0)
#include "system/camerad/imx519_port/cameras/camera_common.h"
#include "system/camerad/imx519_port/cameras/spectra.h"

#include <poll.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#ifdef QCOM2
#include "CL/cl_ext_qcom.h"
#else
#define CL_PRIORITY_HINT_HIGH_QCOM NULL
#define CL_CONTEXT_PRIORITY_HINT_QCOM NULL
#endif

#include "media/cam_sensor_cmn_header.h"

#include "common/clutil.h"
#include "common/params.h"
#include "common/swaglog.h"


ExitHandler do_exit;

// for debugging
const bool env_debug_frames = getenv("DEBUG_FRAMES") != nullptr;
const bool env_log_raw_frames = getenv("LOG_RAW_FRAMES") != nullptr;
const bool env_ctrl_exp_from_params = getenv("CTRL_EXP_FROM_PARAMS") != nullptr;


class CameraState {
public:
  SpectraCamera camera;
  int exposure_time = 1000;  // start with reasonable exposure (was 5, too dark for IMX363)
  bool dc_gain_enabled = false;
  int dc_gain_weight = 0;
  int gain_idx = 0;
  float analog_gain_frac = 0;

  float cur_ev[3] = {};
  float best_ev_score = 0;
  int new_exp_g = 0;
  int new_exp_t = 0;

  Rect ae_xywh = {};
  float measured_grey_fraction = 0;
  float target_grey_fraction = 0.3;

  float fl_pix = 0;
  std::unique_ptr<PubMaster> pm;

  CameraState(SpectraMaster *master, const CameraConfig &config) : camera(master, config) {};
  ~CameraState();
  void init(VisionIpcServer *v, cl_device_id device_id, cl_context ctx);
  void update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain);
  void set_camera_exposure(float grey_frac);
  void set_exposure_rect();
  void sendState();

  float get_gain_factor() const {
    return (1 + dc_gain_weight * (camera.sensor->dc_gain_factor-1) / camera.sensor->dc_gain_max_weight);
  }
};

void CameraState::init(VisionIpcServer *v, cl_device_id device_id, cl_context ctx) {
  camera.camera_open(v, device_id, ctx);

  if (!camera.enabled) return;

  fl_pix = camera.cc.focal_len / camera.sensor->pixel_size_mm * camera.buf.out_img_width/camera.sensor->frame_width;
  set_exposure_rect();

  dc_gain_weight = camera.sensor->dc_gain_min_weight;
  gain_idx = camera.sensor->analog_gain_rec_idx;
  cur_ev[0] = cur_ev[1] = cur_ev[2] = get_gain_factor() * camera.sensor->sensor_analog_gains[gain_idx] * exposure_time;

  if(getenv("IMX519_NATIVE_ACTIVE")) pm=std::make_unique<PubMaster>(std::vector{camera.cc.publish_name});

}

CameraState::~CameraState() {}

void CameraState::set_exposure_rect() {
  // set areas for each camera, shouldn't be changed
  std::vector<std::pair<Rect, float>> ae_targets = {
    // (Rect, F)
    std::make_pair((Rect){96, 250, 1734, 524}, 567.0),  // wide
    std::make_pair((Rect){96, 160, 1734, 986}, 2648.0), // road
    std::make_pair((Rect){96, 242, 1736, 906}, 567.0)   // driver
  };
  int h_ref = 1208;
  /*
    exposure target intrinsics is
    [
      [F, 0, 0.5*ae_xywh[2]]
      [0, F, 0.5*H-ae_xywh[1]]
      [0, 0, 1]
    ]
  */
  auto ae_target = ae_targets[camera.cc.camera_num];
  Rect xywh_ref = ae_target.first;
  float fl_ref = ae_target.second;

  ae_xywh = (Rect){
    std::max(0, (int)camera.buf.out_img_width / 2 - (int)(fl_pix / fl_ref * xywh_ref.w / 2)),
    std::max(0, (int)camera.buf.out_img_height / 2 - (int)(fl_pix / fl_ref * (h_ref / 2 - xywh_ref.y))),
    std::min((int)(fl_pix / fl_ref * xywh_ref.w), (int)camera.buf.out_img_width / 2 + (int)(fl_pix / fl_ref * xywh_ref.w / 2)),
    std::min((int)(fl_pix / fl_ref * xywh_ref.h), (int)camera.buf.out_img_height / 2 + (int)(fl_pix / fl_ref * (h_ref / 2 - xywh_ref.y)))
  };
  // Clamp AE rectangle to actual image dimensions (critical for non-standard resolutions like 960x1080)
  ae_xywh.w = std::min(ae_xywh.w, (int)camera.buf.out_img_width - ae_xywh.x);
  ae_xywh.h = std::min(ae_xywh.h, (int)camera.buf.out_img_height - ae_xywh.y);
  LOGD("AE rect: x=%d y=%d w=%d h=%d (img %dx%d)", ae_xywh.x, ae_xywh.y, ae_xywh.w, ae_xywh.h,
       camera.buf.out_img_width, camera.buf.out_img_height);
}

void CameraState::update_exposure_score(float desired_ev, int exp_t, int exp_g_idx, float exp_gain) {
  float score = camera.sensor->getExposureScore(desired_ev, exp_t, exp_g_idx, exp_gain, gain_idx);
  if (score < best_ev_score) {
    new_exp_t = exp_t;
    new_exp_g = exp_g_idx;
    best_ev_score = score;
  }
}

void CameraState::set_camera_exposure(float grey_frac) {
  if (!camera.enabled) return;
  std::vector<double> target_grey_minimums = {0.1, 0.1, 0.125}; // wide, road, driver

  const auto &sensor = camera.sensor;

  // IMX519 warm-up detection: sensor outputs black frames for ~3-5 seconds after reset.
  // Don't let AE ramp during warm-up, or it will severely overexpose when real data arrives.
  static bool sensor_warmed_up = false;
  static int warmup_frame_count = 0;
  warmup_frame_count++;
  if (!sensor_warmed_up) {
    if (grey_frac < 0.02 && warmup_frame_count < 90) {
      // Still warming up - don't adjust exposure, keep initial settings
      // Timeout after 90 frames (~3 seconds) to handle dim environments
      if (warmup_frame_count % 30 == 0) {
        LOGE("AE WARMUP cam %d frame %u: grey=%.3f (waiting for sensor data...)",
             camera.cc.camera_num, camera.buf.cur_frame_data.frame_id, grey_frac);
      }
      return;
    }
    // First real frame! Set exposure based on current brightness
    sensor_warmed_up = true;
    // Always start with max exposure+gain — AE converges faster downward
    gain_idx = sensor->analog_gain_max_idx;
    exposure_time = sensor->exposure_time_max;
    float init_gain = sensor->sensor_analog_gains[gain_idx];
    float init_ev = (float)exposure_time * init_gain;
    for (int i = 0; i < 3; i++) cur_ev[i] = init_ev;
    LOGE("AE WARMUP DONE cam %d frame %u: grey=%.3f (starting exp=%d gain_idx=%d %.1fx)",
         camera.cc.camera_num, camera.buf.cur_frame_data.frame_id, grey_frac,
         exposure_time, gain_idx, init_gain);
    // Write initial exposure to sensor
    auto warmup_regs = sensor->getExposureRegisters(exposure_time, gain_idx, dc_gain_enabled);
    camera.sensors_i2c(warmup_regs.data(), warmup_regs.size(), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, camera.sensor->data_word);
    return;  // Wait for frame with new settings before normal AE
  }

  // Clamp grey to prevent division by zero and extreme AE oscillation.
  // Instead of forcing extremes (which causes DARK↔BRIGHT ping-pong),
  // let the normal AE algorithm always run with clamped grey values.
  float grey_clamped = std::clamp(grey_frac, 0.005f, 0.995f);
  if (grey_frac < 0.01 || grey_frac > 0.95) {
    LOGE("AE CLAMP cam %d frame %u: grey=%.3f -> clamped to %.3f",
         camera.cc.camera_num, camera.buf.cur_frame_data.frame_id, grey_frac, grey_clamped);
  }

  const float dt = 0.05;

  const float ts_grey = 0.5;
  const float ts_ev = 0.05;

  const float k_grey = (dt / ts_grey) / (1.0 + dt / ts_grey);
  const float k_ev = (dt / ts_ev) / (1.0 + dt / ts_ev);

  // It takes 3 frames for the commanded exposure settings to take effect. The first frame is already started by the time
  // we reach this function, the other 2 are due to the register buffering in the sensor.
  // Therefore we use the target EV from 3 frames ago, the grey fraction that was just measured was the result of that control action.
  // TODO: Lower latency to 2 frames, by using the histogram outputted by the sensor we can do AE before the debayering is complete

  // Offset idx by one to not get stuck in self loop
  const float cur_ev_ = cur_ev[(camera.buf.cur_frame_data.frame_id - 1) % 3] * sensor->ev_scale;

  // Scale target grey between min and 0.4 depending on lighting conditions
  float new_target_grey = std::max<float>(0.45f,target_grey_minimums[camera.cc.camera_num]);
  float target_grey = k_grey*new_target_grey+(1.f-k_grey)*target_grey_fraction;

  float desired_ev = std::clamp(cur_ev_ / sensor->ev_scale * target_grey / grey_clamped, sensor->min_ev, sensor->max_ev);
  float k = (1.0 - k_ev) / 3.0;
  desired_ev = (k * cur_ev[0]) + (k * cur_ev[1]) + (k * cur_ev[2]) + (k_ev * desired_ev);

  best_ev_score = 1e6;
  new_exp_g = 0;
  new_exp_t = 0;

  // Hysteresis around high conversion gain
  // We usually want this on since it results in lower noise, but turn off in very bright day scenes
  bool enable_dc_gain = dc_gain_enabled;
  if (!enable_dc_gain && target_grey < sensor->dc_gain_on_grey) {
    enable_dc_gain = true;
    dc_gain_weight = sensor->dc_gain_min_weight;
  } else if (enable_dc_gain && target_grey > sensor->dc_gain_off_grey) {
    enable_dc_gain = false;
    dc_gain_weight = sensor->dc_gain_max_weight;
  }

  if (enable_dc_gain && dc_gain_weight < sensor->dc_gain_max_weight) {dc_gain_weight += 1;}
  if (!enable_dc_gain && dc_gain_weight > sensor->dc_gain_min_weight) {dc_gain_weight -= 1;}

  std::string gain_bytes, time_bytes;
  if (env_ctrl_exp_from_params) {
    static Params params;
    gain_bytes = params.get("CameraDebugExpGain");
    time_bytes = params.get("CameraDebugExpTime");
  }

  if (gain_bytes.size() > 0 && time_bytes.size() > 0) {
    // Override gain and exposure time
    gain_idx = std::stoi(gain_bytes);
    exposure_time = std::stoi(time_bytes);

    new_exp_g = gain_idx;
    new_exp_t = exposure_time;
    enable_dc_gain = false;
  } else {
    // Simple brute force optimizer to choose sensor parameters to reach desired EV
    int min_g = std::max(gain_idx - 1, sensor->analog_gain_min_idx);
    int max_g = std::min(gain_idx + 1, sensor->analog_gain_max_idx);
    for (int g = min_g; g <= max_g; g++) {
      float gain = sensor->sensor_analog_gains[g] * get_gain_factor();

      // Compute optimal time for given gain
      int t = std::clamp(int(std::round(desired_ev / gain)), sensor->exposure_time_min, sensor->exposure_time_max);

      // Only go below recommended gain when absolutely necessary to not overexpose
      if (g < sensor->analog_gain_rec_idx && t > 20 && g < gain_idx) {
        continue;
      }

      update_exposure_score(desired_ev, t, g, gain);
    }
  }

  measured_grey_fraction = grey_frac;
  target_grey_fraction = target_grey;

  analog_gain_frac = sensor->sensor_analog_gains[new_exp_g];
  gain_idx = new_exp_g;
  exposure_time = new_exp_t;
  dc_gain_enabled = enable_dc_gain;

  float gain = analog_gain_frac * get_gain_factor();
  cur_ev[camera.buf.cur_frame_data.frame_id % 3] = exposure_time * gain;

  // LOGE("ae - camera %d, cur_t %.5f, sof %.5f, dt %.5f", camera.cc.camera_num, 1e-9 * nanos_since_boot(), 1e-9 * camera.buf.cur_frame_data.timestamp_sof, 1e-9 * (nanos_since_boot() - camera.buf.cur_frame_data.timestamp_sof));

  IMX519_TRACE("AE WRITE cam %d frame %u: exp_time=%d gain_idx=%d (%.1fx) grey=%.3f target_grey=%.3f desired_ev=%.1f",
       camera.cc.camera_num, camera.buf.cur_frame_data.frame_id,
       exposure_time, gain_idx, analog_gain_frac, grey_frac, target_grey_fraction, desired_ev);

  auto exp_reg_array = sensor->getExposureRegisters(exposure_time, new_exp_g, dc_gain_enabled);
  camera.sensors_i2c(exp_reg_array.data(), exp_reg_array.size(), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG, camera.sensor->data_word);
}

float imx519_wb_r=1.f, imx519_wb_b=1.f;
static void update_imx519_awb(const CameraBuf &buf) {
  auto *vb=buf.cur_yuv_buf;
  if(!vb || !vb->addr) return;
  const auto *y=(const uint8_t*)vb->addr;
  const auto *uv=y+vb->uv_offset;
  double sr=0,sg=0,sb=0; unsigned count=0;
  for(unsigned row=vb->height/6;row<vb->height*5/6;row+=48) {
    for(unsigned col=vb->width/6;col<vb->width*5/6;col+=48) {
      float l=y[row*vb->stride+col];
      if(l<40 || l>210) continue;
      unsigned k=(row/2)*vb->stride+(col&~1u);
      float u=uv[k]-128.f,v=uv[k+1]-128.f,c=(l-16.f)*1.164f;
      float r=c+1.596f*v,g=c-.392f*u-.813f*v,b=c+2.017f*u;
      if(r<15 || g<15 || b<15 || r>240 || g>240 || b>240) continue;
      sr+=r;sg+=g;sb+=b;++count;
    }
  }
  if(count<40)return;
  const float rr=std::pow(std::clamp((float)(sg/sr),.75f,1.33f),2.2f);
  const float br=std::pow(std::clamp((float)(sg/sb),.75f,1.33f),2.2f);
  imx519_wb_r=std::clamp(imx519_wb_r*(.9f+.1f*rr),.5f,4.f);
  imx519_wb_b=std::clamp(imx519_wb_b*(.9f+.1f*br),.5f,4.f);
}

void CameraState::sendState() {
  auto phase0=nanos_since_boot();
  camera.buf.sendFrameToVipc();
  auto phase1=nanos_since_boot();

  const auto &meta=camera.buf.cur_frame_data;
  const auto now=nanos_since_boot();
  IMX519_TRACE("IMX519_BENCH frame=%u sof_age_ms=%.3f",meta.frame_id,(now-meta.timestamp_sof)/1e6);
  if (getenv("IMX519_SNAPSHOT") && meta.frame_id == 100 && camera.buf.cur_yuv_buf) {
    auto *v=camera.buf.cur_yuv_buf;
    FILE *f=fopen("/data/imx519_bench_frame.nv12","wb");
    if(f) { fwrite(v->addr,1,v->len,f); fclose(f); }
    FILE *j=fopen("/data/imx519_bench_frame.json","w");
    if(j) { fprintf(j,"{\"width\":%u,\"height\":%u,\"stride\":%u,\"uv_offset\":%zu,\"frame_id\":%u}\n",(unsigned)v->width,(unsigned)v->height,(unsigned)v->stride,(size_t)v->uv_offset,meta.frame_id); fclose(j); }
  }
  if(meta.frame_id%4==0) update_imx519_awb(camera.buf);
  if(meta.frame_id%20==0) IMX519_TRACE("IMX519_AWB r=%.3f b=%.3f",imx519_wb_r,imx519_wb_b);
  auto desired=calculate_exposure_value(&camera.buf,ae_xywh,16,16);
  auto phase2=nanos_since_boot();
  set_camera_exposure(desired);
  if(pm) {
    MessageBuilder msg;
    auto out=(msg.initEvent().*camera.cc.init_camera_state)();
    out.setFrameId(meta.frame_id);out.setRequestId(meta.request_id);
    out.setTimestampSof(meta.timestamp_sof);out.setTimestampEof(meta.timestamp_eof);
    out.setIntegLines(exposure_time);out.setGain(analog_gain_frac*get_gain_factor());
    out.setHighConversionGain(false);out.setMeasuredGreyFraction(measured_grey_fraction);
    out.setTargetGreyFraction(target_grey_fraction);out.setProcessingTime(meta.processing_time);
    out.setExposureValPercent(std::clamp(100.f*cur_ev[meta.frame_id%3]/camera.sensor->max_ev,0.f,100.f));
    out.setSensor(camera.sensor->image_sensor);
    pm->send(camera.cc.publish_name,msg);
    static uint64_t last_health=0;
    if(now-last_health>1000000000ULL) {
      FILE *f=fopen("/dev/shm/imx519_native_health.tmp","w");
      if(f) { fprintf(f,"{\"boottime_ns\":%lu,\"frame_id\":%u,\"sof_age_ms\":%.3f,\"geometry\":\"%s\",\"width\":%u,\"height\":%u}\n",now,meta.frame_id,(now-meta.timestamp_sof)/1e6,camera.buf.rectifier?"imx519_720p_20261029":"raw",camera.buf.out_img_width,camera.buf.out_img_height);fclose(f);rename("/dev/shm/imx519_native_health.tmp","/dev/shm/imx519_native_health.json"); }
      last_health=now;
    }
  }

  auto phase3=nanos_since_boot();
  IMX519_TRACE("IMX519_COST publish=%.3f calculate=%.3f apply=%.3f",(phase1-phase0)/1e6,(phase2-phase1)/1e6,(phase3-phase2)/1e6);

}

void camerad_thread() {
  // TODO: centralize enabled handling

  fprintf(stderr,"IMX519 OpenCL device init\n");
  cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  fprintf(stderr,"IMX519 OpenCL device ready\n");
  const cl_context_properties props[] = {CL_CONTEXT_PRIORITY_HINT_QCOM, CL_PRIORITY_HINT_HIGH_QCOM, 0};
  cl_context ctx = CL_CHECK_ERR(clCreateContext(props, 1, &device_id, NULL, NULL, &err));

  fprintf(stderr,"IMX519 OpenCL context ready\n");
  VisionIpcServer v(getenv("IMX519_NATIVE_ACTIVE") ? "camerad" : "imx519_probe", device_id, ctx);

  fprintf(stderr,"IMX519 VisionIPC ready\n");
  if(getenv("IMX519_OPENCL_ONLY")) _exit(0);

  // *** initial ISP init ***
  SpectraMaster m;
  m.init();

  // *** per-cam init ***
  std::vector<std::unique_ptr<CameraState>> cams;
  for (const auto &config : ALL_CAMERA_CONFIGS) {
    auto cam = std::make_unique<CameraState>(&m, config);
    cam->init(&v, device_id, ctx);
    cams.emplace_back(std::move(cam));
  }

  if(std::none_of(cams.begin(),cams.end(),[](auto &c){return c->camera.enabled;})) _exit(4);
  v.start_listener();

  // start devices
  LOG("-- Starting devices");
  for (auto &cam : cams) cam->camera.sensors_start();

  // poll events
  LOG("-- Dequeueing Video events");
  while (!do_exit) {
    struct pollfd fds[1] = {{.fd = m.video0_fd, .events = POLLPRI}};
    int ret = poll(fds, std::size(fds), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    if (!(fds[0].revents & POLLPRI)) continue;

    struct v4l2_event ev = {0};
    ret = HANDLE_EINTR(ioctl(fds[0].fd, VIDIOC_DQEVENT, &ev));
    if (ret == 0) {
      if (ev.type == V4L_EVENT_CAM_REQ_MGR_EVENT) {
        struct cam_req_mgr_message *event_data = (struct cam_req_mgr_message *)ev.u.data;
        if (env_debug_frames) {
          printf("sess_hdl 0x%6X, link_hdl 0x%6X, frame_id %lu, req_id %lu, timestamp %.2f ms, sof_status %d\n", event_data->session_hdl, event_data->u.frame_msg.link_hdl,
                 event_data->u.frame_msg.frame_id, event_data->u.frame_msg.request_id, event_data->u.frame_msg.timestamp/1e6, event_data->u.frame_msg.sof_status);
          do_exit = do_exit || event_data->u.frame_msg.frame_id > (20*20);
        }

        for (auto &cam : cams) {
          if (event_data->session_hdl == cam->camera.session_handle) {
            if (cam->camera.handle_camera_event(event_data)) {
              cam->sendState();
            }
            break;
          }
        }
      } else {
        LOGE("unhandled event %d\n", ev.type);
      }
    } else {
      LOGE("VIDIOC_DQEVENT failed, errno=%d", errno);
    }
  }
  _exit(0);

}
