#pragma once

#include "media/cam_sensor.h"

// IMX363 start streaming register
const struct i2c_random_wr_payload start_reg_array_imx363[] = {
  {0x0100, 0x01},  // MODE_SELECT = streaming
};

// IMX363 init register array
// Mode: 960x1080 crop @ ~13fps, 10-bit RAW RGGB, 4 MIPI lanes
// MCLK: 24 MHz
// PLL VCO: 1248 MHz, VT_PIX_CLK: 208 MHz
// MIPI: OP_VCO = 1380 MHz, 138 MHz/lane DDR
//
// Restored from MIUI vendor config (Day 7 working state)
// DO NOT remove vendor registers - they control MIPI TX and analog frontend
const struct i2c_random_wr_payload init_array_imx363[] = {
  // ============================================================
  // Standby (NO software reset - 0x0103 causes NACK on subsequent
  // writes because sensor needs ~1ms recovery but CCI FIFO sends
  // next command immediately. MIUI driver doesn't use SW reset.)
  // ============================================================
  {0x0100, 0x00},  // MODE_SELECT = standby

  // External clock frequency (24.0 MHz)
  {0x0136, 0x18},  // EXCK_FREQ [15:8] = 24
  {0x0137, 0x00},  // EXCK_FREQ [7:0] = .0

  // ============================================================
  // GROUP A: Global init (initSettings id=1939)
  // Calibration and internal signal processing
  // ============================================================
  {0x31A3, 0x00},
  {0x64D4, 0x01}, {0x64D5, 0xAA}, {0x64D6, 0x01}, {0x64D7, 0xA9},
  {0x64D8, 0x01}, {0x64D9, 0xA5}, {0x64DA, 0x01}, {0x64DB, 0xA1},
  {0x720A, 0x24}, {0x720B, 0x89}, {0x720C, 0x85}, {0x720D, 0xA1}, {0x720E, 0x6E},
  {0x729C, 0x59},
  {0x817C, 0xFF}, {0x817D, 0x80},
  {0x9348, 0x96}, {0x934B, 0x8C}, {0x934C, 0x82},
  {0x9353, 0xAA}, {0x9354, 0xAA},

  // ============================================================
  // GROUP B: Pre-init (regSetting id=1906)
  // Internal power, ADC, data path initialization
  // ============================================================
  {0x30A1, 0x00},
  {0x4B5F, 0x00},
  {0x4B66, 0xFF},
  {0x4B70, 0x00},
  {0x4BD0, 0x00},
  {0x5872, 0x00},
  {0x5873, 0x0C},
  {0x5875, 0x00},
  {0x5D0C, 0x00},
  {0x5D0D, 0x01},

  // ============================================================
  // GROUP C: Per-mode vendor regs (Mode 0 full-res)
  // 0x31A0: MIUI uses 0x01 (NOT 0x00). 0x00 blocks exposure control.
  // TEST: Removed 0x30F4-0x30F7 to isolate bad frame cause
  // ============================================================
  // {0x30F4, 0x02}, {0x30F5, 0x80}, {0x30F6, 0x00}, {0x30F7, 0xC8},
  {0x31A0, 0x01}, {0x31A5, 0x00}, {0x31A6, 0x00},
  // C2: ADC/signal/output/MIPI
  {0x560F, 0xBE}, {0x5856, 0x08}, {0x58D0, 0x10},
  {0x734A, 0x01}, {0x734F, 0x2B}, {0x7441, 0x55},
  {0x7914, 0x03}, {0x7928, 0x04}, {0x7929, 0x04},
  {0x793F, 0x03}, {0xBC7B, 0x18},

  // ============================================================
  // Data format - 10-bit RAW, 4 MIPI lanes
  // ============================================================
  {0x0112, 0x0A},  // CSI_DATA_FORMAT [15:8]
  {0x0113, 0x0A},  // CSI_DATA_FORMAT [7:0]
  {0x0114, 0x03},  // CSI_LANE_MODE = 4 lanes

  // ============================================================
  // PLL settings (from MIUI Mode 0 - full resolution)
  // VT: VCO = 24/4 * 208 = 1248 MHz
  //     VT_SYS = 1248/2 = 624 MHz
  //     VT_PIX = 624/3 = 208 MHz
  // OP: VCO = 24/4 * 230 = 1380 MHz
  //     OP_SYS = 1380/1 = 1380 MHz
  //     OP_PIX = 1380/10 = 138 MHz/lane (276 Mbps DDR/lane)
  // ============================================================
  {0x0301, 0x03},  // VT_PIX_CLK_DIV = 3
  {0x0303, 0x02},  // VT_SYS_CLK_DIV = 2
  {0x0305, 0x04},  // PRE_PLL_CLK_DIV = 4
  {0x0306, 0x00},  // PLL_MULTIPLIER [12:8]
  {0x0307, 0xD0},  // PLL_MULTIPLIER [7:0] = 208

  {0x0309, 0x0A},  // OP_PIX_CLK_DIV = 10
  {0x030B, 0x01},  // OP_SYS_CLK_DIV = 1

  {0x030D, 0x04},  // OP_PRE_PLL_CLK_DIV = 4
  {0x030E, 0x00},  // OP_PLL_MULTIPLIER [10:8]
  {0x030F, 0xE6},  // OP_PLL_MULTIPLIER [7:0] = 230
  {0x0310, 0x01},  // PLL mode

  // ============================================================
  // Frame timing
  // Full-array readout: need FLL >= 3024, LLP >= ~4200
  // LINE_LENGTH = 5024, FRAME_LENGTH = 3100
  // FPS = 208 MHz / (5024 * 3100) = ~13.4 fps
  // ============================================================
  {0x0342, 0x13},  // LINE_LENGTH_PCK [15:8] = 5024 (0x13A0)
  {0x0343, 0xA0},  // LINE_LENGTH_PCK [7:0]
  {0x0340, 0x0D},  // FRAME_LENGTH_LINES [15:8] = 3500 (0x0DAC) ~47fps@VT_PIX≈832MHz
  {0x0341, 0xAC},  // FRAME_LENGTH_LINES [7:0]

  // ============================================================
  // Analog crop window: FULL ARRAY readout (Mode 0 compatible)
  // X: 0 to 4031 (4032 pixels)
  // Y: 0 to 3023 (3024 pixels)
  // This matches the vendor registers from MIUI Mode 0
  // ============================================================
  {0x0344, 0x00},  // X_ADD_STA [12:8] = 0
  {0x0345, 0x00},  // X_ADD_STA [7:0]
  {0x0346, 0x00},  // Y_ADD_STA [12:8] = 0
  {0x0347, 0x00},  // Y_ADD_STA [7:0]
  {0x0348, 0x0F},  // X_ADD_END [12:8] = 4031 (0x0FBF)
  {0x0349, 0xBF},  // X_ADD_END [7:0]
  {0x034A, 0x0B},  // Y_ADD_END [12:8] = 3023 (0x0BCF)
  {0x034B, 0xCF},  // Y_ADD_END [7:0]

  // Output size = 960x1080 (after digital crop)
  {0x034C, 0x03},  // X_OUTPUT_SIZE [12:8] = 960 (0x03C0)
  {0x034D, 0xC0},  // X_OUTPUT_SIZE [7:0]
  {0x034E, 0x04},  // Y_OUTPUT_SIZE [12:8] = 1080 (0x0438)
  {0x034F, 0x38},  // Y_OUTPUT_SIZE [7:0]

  // ============================================================
  // Subsampling / binning (OFF for full readout + digital crop)
  // ============================================================
  {0x0220, 0x00},
  {0x0221, 0x11},
  {0x0381, 0x01},  // X_EVN_INC
  {0x0383, 0x01},  // X_ODD_INC
  {0x0385, 0x01},  // Y_EVN_INC
  {0x0387, 0x01},  // Y_ODD_INC
  {0x0900, 0x00},  // BINNING_MODE = OFF
  {0x0901, 0x11},  // BINNING_TYPE H=1, V=1

  // ============================================================
  // Digital crop: crop center 960x1080 from 4032x3024 readout
  // X offset = (4032-960)/2 = 1536
  // Y offset = (3024-1080)/2 = 972
  // ============================================================
  {0x0401, 0x00},  // SCALING_MODE = no scaling
  {0x0404, 0x00},
  {0x0405, 0x10},  // SCALE_M = 16 (1:1)
  {0x0408, 0x06},  // DIG_CROP_X_OFFSET [12:8] = 1536 (0x0600)
  {0x0409, 0x00},  // DIG_CROP_X_OFFSET [7:0]
  {0x040A, 0x03},  // DIG_CROP_Y_OFFSET [12:8] = 972 (0x03CC)
  {0x040B, 0xCC},  // DIG_CROP_Y_OFFSET [7:0]
  {0x040C, 0x03},  // DIG_CROP_IMAGE_WIDTH = 960
  {0x040D, 0xC0},
  {0x040E, 0x04},  // DIG_CROP_IMAGE_HEIGHT = 1080
  {0x040F, 0x38},

  // ============================================================
  // Exposure defaults
  // ============================================================
  {0x0202, 0x01},  // COARSE_INTEG_TIME [15:8] = 500 (0x01F4)
  {0x0203, 0xF4},  // COARSE_INTEG_TIME [7:0]
  {0x0224, 0x01},  // Short exposure
  {0x0225, 0xF4},
  {0x0204, 0x00},  // ANA_GAIN_GLOBAL [15:8] = 0 (1x)
  {0x0205, 0x00},  // ANA_GAIN_GLOBAL [7:0]
  {0x0216, 0x00},  // Short gain
  {0x0217, 0x00},

  // Digital gain (all channels 1x = 0x0100)
  {0x020E, 0x01},  // DIG_GAIN_GR [15:8]
  {0x020F, 0x00},  // DIG_GAIN_GR [7:0]
  {0x0210, 0x01},  // DIG_GAIN_R [15:8]
  {0x0211, 0x00},  // DIG_GAIN_R [7:0]
  {0x0212, 0x01},  // DIG_GAIN_B [15:8]
  {0x0213, 0x00},  // DIG_GAIN_B [7:0]
  {0x0214, 0x01},  // DIG_GAIN_GB [15:8]
  {0x0215, 0x00},  // DIG_GAIN_GB [7:0]
  {0x0226, 0x01},  // Short digital gain
  {0x0227, 0x00},

  // ============================================================
  // Image orientation (no flip)
  // ============================================================
  {0x0101, 0x00},

  // ============================================================
  // Test pattern: OFF for real image diagnostic
  // 0x00=OFF, 0x01=Solid Color, 0x02=Color Bar, 0x03=Grey Ramp
  // ============================================================
  {0x0600, 0x00},
  {0x0601, 0x00},  // Test pattern = OFF (real sensor data)
  // Solid color test data (used when 0x0601=0x01)
  {0x0602, 0x03},  // TEST_DATA_R [15:8] = 0x03FF (max 10-bit)
  {0x0603, 0xFF},
  {0x0604, 0x02},  // TEST_DATA_GR [15:8] = 0x0200
  {0x0605, 0x00},
  {0x0606, 0x01},  // TEST_DATA_B [15:8] = 0x0100
  {0x0607, 0x00},
  {0x0608, 0x02},  // TEST_DATA_GB [15:8] = 0x0200
  {0x0609, 0x00}
};
