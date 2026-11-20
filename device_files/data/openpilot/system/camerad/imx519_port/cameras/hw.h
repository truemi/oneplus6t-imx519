#pragma once

#include "common/util.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "msgq/visionipc/visionipc_server.h"

#include "media/cam_isp_ife.h"


typedef enum {
  ISP_RAW_OUTPUT,   // raw frame from sensor
  ISP_IFE_PROCESSED,  // fully processed image through the IFE
  ISP_BPS_PROCESSED,  // fully processed image through the BPS
} SpectraOutputType;

// OnePlus 6T (fajita), native IMX519 main camera profile

struct CameraConfig {
  int camera_num;
  VisionStreamType stream_type;
  float focal_len;  // millimeters
  const char *publish_name;
  cereal::FrameData::Builder (cereal::Event::Builder::*init_camera_state)();
  bool enabled;
  uint32_t phy;
  bool vignetting_correction;
  SpectraOutputType output_type;
};

// OnePlus 6T: physical camera 0 = IMX519 rear main (CSIPHY 0)
// Use as ROAD camera for driving
const CameraConfig ROAD_CAMERA_CONFIG = {
  .camera_num = 0,
  .stream_type = VISION_STREAM_ROAD,
  .focal_len = 4.25,
  .publish_name = "roadCameraState",
  .init_camera_state = &cereal::Event::Builder::initRoadCameraState,
  .enabled = !getenv("DISABLE_ROAD"),
  .phy = CAM_ISP_IFE_IN_RES_PHY_0,
  .vignetting_correction = false,
  .output_type = ISP_IFE_PROCESSED,  // Use hardware ISP for demosaic+color+gamma
};

// Disabled wide placeholder; do not duplicate the main view as an ultra-wide camera
const CameraConfig WIDE_ROAD_CAMERA_CONFIG = {
  .camera_num = 0,
  .stream_type = VISION_STREAM_WIDE_ROAD,
  .focal_len = 4.25,
  .publish_name = "wideRoadCameraState",
  .init_camera_state = &cereal::Event::Builder::initWideRoadCameraState,
  .enabled = false,  // disabled - single rear camera only
  .phy = CAM_ISP_IFE_IN_RES_PHY_0,
  .vignetting_correction = false,
  .output_type = ISP_IFE_PROCESSED,
};

// Disabled front placeholder; the OnePlus 6T front sensor is IMX371
// Use as DRIVER camera for driver monitoring
const CameraConfig DRIVER_CAMERA_CONFIG = {
  .camera_num = 2,
  .stream_type = VISION_STREAM_DRIVER,
  .focal_len = 3.67,
  .publish_name = "driverCameraState",
  .init_camera_state = &cereal::Event::Builder::initDriverCameraState,
  .enabled = false,  // disabled for now - front sensor is not implemented in this backend
  .phy = CAM_ISP_IFE_IN_RES_PHY_2,
  .vignetting_correction = false,
  .output_type = ISP_BPS_PROCESSED,
};

const CameraConfig ALL_CAMERA_CONFIGS[] = {ROAD_CAMERA_CONFIG, WIDE_ROAD_CAMERA_CONFIG, DRIVER_CAMERA_CONFIG};
