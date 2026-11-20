#include <algorithm>
#include <cmath>
#include "system/camerad/imx519_port/sensors/sensor.h"
#include "system/camerad/imx519_port/sensors/imx519_registers.h"

IMX519::IMX519() {
  image_sensor = cereal::FrameData::ImageSensor::UNKNOWN;
  pixel_size_mm = 0.00244f; // effective pitch after native 2x2 binning
  frame_width = 1920; frame_height = 1080; frame_stride = 2400;
  out_scale = 1; data_word = false;
  probe_reg_addr = 0x0016; probe_expected_data = 0x0519;
  bits_per_pixel = 10; bayer_pattern = CAM_ISP_PATTERN_BAYER_RGRGRG;
  mipi_format = CAM_FORMAT_MIPI_RAW_10; mclk_frequency = 24000000; frame_data_type = 0x2b;
  // Provisional readout estimate, never use as measured EOF latency.
  readout_time_ns = 8500000;
  exposure_time_min = 2; exposure_time_max = 7471 - 16;
  dc_gain_factor = 1; dc_gain_min_weight = 1; dc_gain_max_weight = 1;
  dc_gain_on_grey = .9; dc_gain_off_grey = 1.;
  analog_gain_min_idx=0; analog_gain_max_idx=15; analog_gain_rec_idx=0;
  analog_gain_cost_delta=-1; analog_gain_cost_low=.4; analog_gain_cost_high=6.4;
  for (int i=0; i<=15; ++i) sensor_analog_gains[i] = 1024.f/(1024.f-i*64);
  target_grey_factor=0.01; min_ev=2; max_ev=exposure_time_max*sensor_analog_gains[15];
  start_reg_array={{0x0100,1}};
  init_reg_array.assign(std::begin(imx519_init),std::end(imx519_init));
  // Generic gamma/neutral LSC plus a factory D65 CCM candidate; not full OEM tuning.
  black_level=64;
  color_correct_matrix={179,4070,4071,4021,200,3,4038,4084,198};
  for (int i=0;i<=64;++i) gamma_lut_rgb.push_back((uint32_t)std::round(1023*std::pow(i/64.f,1/2.2f)));
  prepare_gamma_lut();
  linearization_lut={0x02000000,0x02000000,0x02000000,0x02000000,
    0x0200003f,0x0200003f,0x0200003f,0x0200003f,
    0x020007bf,0x020007bf,0x020007bf,0x020007bf,
    0x02000f3f,0x02000f3f,0x02000f3f,0x02000f3f,
    0x020016bf,0x020016bf,0x020016bf,0x020016bf,
    0x02001e3f,0x02001e3f,0x02001e3f,0x02001e3f,
    0x020025bf,0x020025bf,0x020025bf,0x020025bf,
    0x02002d3f,0x02002d3f,0x02002d3f,0x02002d3f,
    0x020034bf,0x020034bf,0x020034bf,0x020034bf};
  linearization_pts={0x003f07bf,0x0f3f16bf,0x1e3f25bf,0x2d3f34bf};
  vignetting_lut.assign(221,(1024u<<13)|1024u);
}
std::vector<i2c_random_wr_payload> IMX519::getExposureRegisters(int exposure_time,int gain,bool) const {
  const unsigned t=std::clamp(exposure_time,exposure_time_min,exposure_time_max);
  const unsigned g=std::clamp(gain,0,15)*64;
  return {{0x0104,1},{0x0202,(uint16_t)(t>>8)},{0x0203,(uint16_t)(t&255)},
          {0x0204,(uint16_t)(g>>8)},{0x0205,(uint16_t)(g&255)},
          {0x020e,1},{0x020f,0},{0x0104,0}};
}
int IMX519::getSlaveAddress(int port) const { assert(port==0); return 0x34; }
float IMX519::getExposureScore(float ev,int t,int g,float gain,int previous) const {
  return std::abs(ev-t*gain)+std::abs(g-previous)*3.f+g*.4f;
}
