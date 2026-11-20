#include <cmath>

#include "system/camerad/imx519_port/sensors/sensor.h"

namespace {

// Sony IMX analog gain: gain = 1024 / (1024 - code)
// Code range: 0 to 960, step 64
const float sensor_analog_gains_IMX363[] = {
    1.000,   // code 0
    1.067,   // code 64
    1.143,   // code 128
    1.231,   // code 192
    1.333,   // code 256
    1.455,   // code 320
    1.600,   // code 384
    1.778,   // code 448
    2.000,   // code 512
    2.286,   // code 576
    2.667,   // code 640
    3.200,   // code 704
    4.000,   // code 768
    5.333,   // code 832
    8.000,   // code 896
    16.000,  // code 960
};

__attribute__((unused))
const uint32_t imx363_analog_gains_reg[] = {
    0x000,  // 1.000x
    0x040,  // 1.067x
    0x080,  // 1.143x
    0x0C0,  // 1.231x
    0x100,  // 1.333x
    0x140,  // 1.455x
    0x180,  // 1.600x
    0x1C0,  // 1.778x
    0x200,  // 2.000x
    0x240,  // 2.286x
    0x280,  // 2.667x
    0x2C0,  // 3.200x
    0x300,  // 4.000x
    0x340,  // 5.333x
    0x380,  // 8.000x
    0x3C0,  // 16.000x
};

}  // namespace

IMX363::IMX363() {
  image_sensor = cereal::FrameData::ImageSensor::UNKNOWN;  // TODO: add IMX363 to cereal enum
  bayer_pattern = CAM_ISP_PATTERN_BAYER_RGRGRG;  // IMX363 = RGGB
  pixel_size_mm = 0.0014;  // 1.4 um pixel pitch
  data_word = false;

  frame_width = 3840;
  frame_height = 2160;
  frame_stride = (frame_width * 10 / 8);  // 4800 bytes for RAW10
  out_scale = 2;  // GPU downsamples 3840→1920, 2160→1080

  extra_height = 0;
  frame_offset = 0;

  start_reg_array.assign(std::begin(start_reg_array_imx363), std::end(start_reg_array_imx363));
  init_reg_array.assign(std::begin(init_array_imx363), std::end(init_array_imx363));

  probe_reg_addr = 0x0016;       // Chip ID register (standard Sony IMX)
  probe_expected_data = 0x0363;  // IMX363 chip ID
  bits_per_pixel = 10;
  mipi_format = CAM_FORMAT_MIPI_RAW_10;
  frame_data_type = 0x2B;       // MIPI CSI-2 RAW10 data type
  mclk_frequency = 24000000;    // 24 MHz

  readout_time_ns = 15000000;   // ~15ms estimate

  ev_scale = 1.0;
  dc_gain_factor = 1;
  dc_gain_min_weight = 1;
  dc_gain_max_weight = 1;
  dc_gain_on_grey = 0.9;
  dc_gain_off_grey = 1.0;

  exposure_time_min = 2;
  exposure_time_max = 3124;  // FLL(3140) - 16 (full-res 3840x2160 30fps)

  analog_gain_min_idx = 0;
  analog_gain_rec_idx = 0;    // 1x
  analog_gain_max_idx = 15;   // Analog gain up to 8.0x (code 0x380)   // DIG_GAIN caps at 0x0FFF (16x) at base×4.0×4.0=16x
  analog_gain_cost_delta = -1;
  analog_gain_cost_low = 0.4;
  analog_gain_cost_high = 6.4;

  for (int i = 0; i <= analog_gain_max_idx; i++) {
    sensor_analog_gains[i] = sensor_analog_gains_IMX363[i];
  }

  min_ev = exposure_time_min * sensor_analog_gains[analog_gain_min_idx];
  max_ev = exposure_time_max * dc_gain_factor * sensor_analog_gains[analog_gain_max_idx];
  target_grey_factor = 0.01;

  black_level = 64;  // typical for 10-bit Sony IMX

  // Color correction matrix - D65 pure Chromatix, GBR channel order
  color_correct_matrix = {
    0x000000c1, 0x00000fd6, 0x00000fe9,  // G_out: [G*1.509, B*-0.331, R*-0.178]
    0x00000fa0, 0x000000d9, 0x00000008,  // B_out: [G*-0.753, B*1.693, R*0.060]
    0x00000fa5, 0x00000ff9, 0x000000e2,  // R_out: [G*-0.712, B*-0.052, R*1.763]
  };

  // Gamma LUT from Chromatix factory calibration (258→65 downsampled)
  gamma_lut_rgb = {
    0, 36, 84, 132, 180, 231, 275, 319, 359, 398, 434, 463, 490,
    514, 536, 556, 575, 594, 613, 631, 648, 664, 678, 692, 705, 717,
    729, 741, 752, 763, 773, 784, 794, 803, 813, 822, 831, 840, 848,
    856, 865, 872, 880, 888, 895, 903, 911, 917, 924, 931, 937, 943,
    950, 956, 963, 968, 975, 980, 987, 992, 999, 1004, 1010, 1016, 1022,
  };
  prepare_gamma_lut();

  // Linearization LUT — IMX363 linear sensor, knee points from Chromatix
  linearization_lut = {
    0x02000000, 0x02000000, 0x02000000, 0x02000000,  // seg 0
    0x0200003f, 0x0200003f, 0x0200003f, 0x0200003f,  // seg 1
    0x020007bf, 0x020007bf, 0x020007bf, 0x020007bf,  // seg 2
    0x02000f3f, 0x02000f3f, 0x02000f3f, 0x02000f3f,  // seg 3
    0x020016bf, 0x020016bf, 0x020016bf, 0x020016bf,  // seg 4
    0x02001e3f, 0x02001e3f, 0x02001e3f, 0x02001e3f,  // seg 5
    0x00003fff, 0x00003fff, 0x00003fff, 0x00003fff,  // seg 6
    0x00003fff, 0x00003fff, 0x00003fff, 0x00003fff,  // seg 7
    0x00003fff, 0x00003fff, 0x00003fff, 0x00003fff,  // seg 8
  };
  linearization_pts = {0x003f07bf, 0x0f3f16bf, 0x1e3f25bf, 0x2d3f34bf};

  // Vignetting LUT (GRR bank) — LSC from Chromatix Semco IMX363
  // 13×17 mesh, packed {gr_gain[25:13], r_gain[12:0]} UQ3.10
  vignetting_lut = {
    0x01de52b2, 0x01928fb3, 0x0156ed5a, 0x012e2bd6, 0x01164aec, 0x0102ca18, 0x00f42971, 0x00eac905, 0x00e728db, 0x00e908f3, 0x00f06948, 0x00fce9dc, 0x010f8aa9, 0x01264b8e, 0x014a0ce5, 0x0181af0b, 0x01c911be,
    0x01a67084, 0x01612dbc, 0x0130abe8, 0x01128abe, 0x00f9c9b4, 0x00e568cc, 0x00d7a823, 0x00cfa7bd, 0x00cca793, 0x00ce67ac, 0x00d50804, 0x00e0a896, 0x00f2c968, 0x010b0a70, 0x01274b88, 0x01532d30, 0x01986fc4,
    0x017aeec5, 0x013dec6c, 0x0118eb06, 0x00fb69cb, 0x00e188a5, 0x00cee7c0, 0x00c08708, 0x00b7468b, 0x00b3a65b, 0x00b5c67b, 0x00bdc6e4, 0x00ca878c, 0x00db485d, 0x00f30972, 0x0110aaab, 0x01330bfb, 0x016e4e1d,
    0x015bad91, 0x01280b98, 0x01076a51, 0x00e808f6, 0x00d007d2, 0x00bc26d5, 0x00aba603, 0x00a1457f, 0x009de550, 0x00a06571, 0x00a8c5de, 0x00b7469a, 0x00ca278c, 0x00e0089b, 0x00fdc9e5, 0x011ecb34, 0x01500cf4,
    0x0146acce, 0x011b2b1f, 0x00fa29c2, 0x00dae85e, 0x00c2c730, 0x00ad0619, 0x009bc542, 0x0091c4c4, 0x008e0495, 0x0090c4b6, 0x0099c526, 0x00a8a5e3, 0x00bce6e8, 0x00d3a809, 0x00f0294e, 0x01120aba, 0x013d0c45,
    0x013ccc72, 0x01138ad4, 0x00f1c96b, 0x00d36805, 0x00ba86c7, 0x00a3e5a8, 0x009284d3, 0x0087e454, 0x0083c425, 0x0086a446, 0x009044b6, 0x009fc575, 0x00b4c67f, 0x00cc87b2, 0x00e808f7, 0x010a2a67, 0x01336bee,
    0x0139ac58, 0x01116abc, 0x00efa951, 0x00d147eb, 0x00b846a9, 0x00a10586, 0x009004b7, 0x00850436, 0x0080e407, 0x0083e428, 0x008d8498, 0x009d4557, 0x00b26663, 0x00ca2797, 0x00e5a8de, 0x01082a53, 0x0130ebd2,
    0x013dac7a, 0x01146ada, 0x00f30976, 0x00d46812, 0x00bbe6da, 0x00a4e5b9, 0x0093e4e6, 0x0089a46a, 0x0085643a, 0x0088245a, 0x0091a4ca, 0x00a1058a, 0x00b56691, 0x00cd27c0, 0x00e88904, 0x010aea72, 0x0133ebf1,
    0x014a8ced, 0x011c8b27, 0x00fc49d8, 0x00dd2878, 0x00c5474f, 0x00afe63e, 0x009ea569, 0x0094c4eb, 0x009104bd, 0x0093c4df, 0x009c454e, 0x00aac609, 0x00bec70a, 0x00d4e822, 0x00f1a968, 0x0112cac3, 0x013f2c5b,
    0x0160edc8, 0x012b6bb7, 0x010aaa70, 0x00ebc91e, 0x00d347fa, 0x00bfe708, 0x00b08640, 0x00a665bf, 0x00a2c592, 0x00a505b1, 0x00acc61d, 0x00baa6d6, 0x00cc47bb, 0x00e268c4, 0x00ffea07, 0x01202b46, 0x0153ad1f,
    0x01868f4b, 0x0144ecb0, 0x011ceb28, 0x01006a03, 0x00e6e8e4, 0x00d3c801, 0x00c64752, 0x00bd66e0, 0x00ba06b5, 0x00bbe6d2, 0x00c30735, 0x00ce47cf, 0x00df289d, 0x00f6c9ad, 0x0112aacf, 0x0136ac26, 0x01740e58,
    0x01bcb190, 0x016d2e3d, 0x01386c2e, 0x0118caf6, 0x01012a03, 0x00ecc922, 0x00df087e, 0x00d6a817, 0x00d347f2, 0x00d4a808, 0x00dac858, 0x00e648e9, 0x00f8c9bc, 0x010eeaa6, 0x012acbba, 0x01588d77, 0x01a2103d,
    0x0206d44b, 0x01aa3085, 0x0166cdd5, 0x013aec2b, 0x0120ab35, 0x010eaa76, 0x00ffe9d7, 0x00f62973, 0x00f28948, 0x00f3a95d, 0x00fb89b5, 0x0107ca3f, 0x0118caf7, 0x012e8bca, 0x0155ad3b, 0x018f6f8e, 0x01dc129c,
  };
}

std::vector<i2c_random_wr_payload> IMX363::getExposureRegisters(int exposure_time, int new_exp_g, bool dc_gain_enabled) const {
  // Use true analog gain for lower noise. Digital gain fixed at 1x.
  // Analog gain register codes: gain = 1024/(1024-code), init_array confirms register works.
  uint16_t ana_gain_code = imx363_analog_gains_reg[new_exp_g];

  return {
    {0x0104, 0x01},  // GROUPED_PARAMETER_HOLD = ON
    {0x0202, (uint16_t)(exposure_time >> 8)},
    {0x0203, (uint16_t)(exposure_time & 0xFF)},
    {0x0204, (uint16_t)(ana_gain_code >> 8)}, {0x0205, (uint16_t)(ana_gain_code & 0xFF)},
    {0x020E, 0x01}, {0x020F, 0x00},  // DIG_GAIN = 1.0x (256/256)
    {0x0104, 0x00},  // GROUPED_PARAMETER_HOLD = OFF (apply)
  };
}

int IMX363::getSlaveAddress(int port) const {
  assert(port >= 0 && port <= 2);
  // IMX363 rear main camera on Xiaomi Mi 8: I2C write address 0x34
  return (int[]){0x34, 0x34, 0x34}[port];
}

float IMX363::getExposureScore(float desired_ev, int exp_t, int exp_g_idx, float exp_gain, int gain_idx) const {
  float score = std::abs(desired_ev - (exp_t * exp_gain));
  float m = exp_g_idx > analog_gain_rec_idx ? analog_gain_cost_high : analog_gain_cost_low;
  score += std::abs(exp_g_idx - (int)analog_gain_rec_idx) * m;
  score += ((1 - analog_gain_cost_delta) +
            analog_gain_cost_delta * (exp_g_idx - analog_gain_min_idx) / (analog_gain_max_idx - analog_gain_min_idx)) *
           std::abs(exp_g_idx - gain_idx) * 3.0;
  return score;
}
