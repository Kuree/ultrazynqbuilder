int COMMANDS_LEN = 72;

uint8_t byte_strings[][5] = { // 72 lines
{3,0x01,0x03,0x01,0}, // SW_RESET
{3,0x01,0x00,0x00,0}, // MODE_SEL: standby
{3,0x01,0x00,0x00,0}, // MODE_SEL: standby (again?)
{4,0x66,0x20,0x01,0x01}, // ???
{4,0x66,0x22,0x01,0x01}, // ???
{3,0x30,0xEB,0x0C,0}, // Have to send this magic access code to get to one-time-programmable memory (but why?)
{3,0x30,0xEB,0x05,0}, // I thought this byte had to go first?
{4,0x30,0x0A,0xFF,0xFF},
{3,0x30,0xEB,0x05,0},
{3,0x30,0xEB,0x09,0}, // End of the secret access code
{3,0x01,0x14,0x01,0},    // CSI_LANE_MODE: 2 lanes
{3,0x01,0x28,0x00,0},    // DPHY_CTRL: auto mode
{4,0x01,0x2A,0x18,0x00}, // EXCK_FREQ = 0x1800 [MHz??]
{3,0x01,0x57,0x00,0},    // ANA_GAIN_GLOBAL_A: no analog gain?
{4,0x01,0x5A,0x09,0xBD}, // COARSE_INTEGRATION_TIME_A
{4,0x01,0x60,0x03,0x72}, // FRM_LENGTH_A: 0x372 = 882 (???)
{4,0x01,0x62,0x0D,0x78}, // LINE_LENGTH_A: 0xD78 = 3448
{4,0x01,0x64,0x00,0x00}, // X_ADD_STA_A: 0 (left edge)
{4,0x01,0x66,0x0C,0xCF}, // X_ADD_END_A: 0xCCF = 3279 (right edge)
{4,0x01,0x68,0x00,0x00}, // Y_ADD_STA_A: 0 (top edge)
{4,0x01,0x6A,0x09,0x9F}, // Y_ADD_END_A: 0x99f = 2463 (bottom edge)
{4,0x01,0x6C,0x06,0x68}, // x_output_size: 0x668 = 1640 (???)
{4,0x01,0x6E,0x04,0xD0}, // y_output_size: 0x4d0 = 1232 (???)
{4,0x01,0x70,0x01,0x01}, // X_ODD_INC_A: 1, Y_ODD_INC_A: 1
{4,0x01,0x74,0x01,0x01}, // BINNING_MODE_H_A: 1 (x2 binning), BINNING_MODE_V_A: 1 (x2 binning)
{4,0x01,0x8C,0x0A,0x0A}, // CSI_DATA_FORMAT_A: 0x0A0A for RAW 10 (0x0808 for RAW 8)
{3,0x03,0x01,0x05,0},    // VTPXCK_DIV (Video timing clock divider): 0x05 (5 bits)
{4,0x03,0x03,0x01,0x03}, // VTSYCK_DIV (Video timing system clock div): 0x01 (2 bits)
                         // PREPLLCK_VT_DIV (Pre PLL clock Video Timing System Divider): 0x03 (8 bits)
{4,0x03,0x05,0x03,0x00}, // PREPLLCK_OP_DIV (Pre PLL clock Output System Divider Value): 0x03
                         // PLL_VT_MPY (PLL Video Timing System multiplier): 0 (high 3 bits)
{3,0x03,0x07,0x39,0},    // PLL_VT_MPY (PLL Video Timing System multiplier): 0x39 (low 8 bits)
{3,0x03,0x09,0x0A,0},    // OPPXCK_DIV (Output Pixel Clock Divider): 0x0A = 10
{4,0x03,0x0B,0x01,0x00}, // OPSYCK_DIV (Output System Clock Divider): 0x01
                         // PLL_OP_MPY (top 3 bits): 0
{3,0x03,0x0D,0x72,0},    // PLL_OP_MPY (PLL Output System multiplier): 0x72
{3,0x45,0x5E,0x00,0}, // No idea...
{3,0x47,0x1E,0x4B,0},
{3,0x47,0x67,0x0F,0},
{3,0x47,0x50,0x14,0},
{3,0x45,0x40,0x00,0},
{3,0x47,0xB4,0x14,0},
{3,0x47,0x13,0x30,0},
{3,0x47,0x8B,0x10,0},
{3,0x47,0x8F,0x10,0},
{3,0x47,0x93,0x10,0},
{3,0x47,0x97,0x0E,0},
{3,0x47,0x9B,0x0E,0},
{3,0x01,0x72,0x03,0},    // IMG_ORIENTATION_A (vertical and horizontal): 0b11
{4,0x01,0x60,0x06,0xE3}, // FRM_LENGTH_A: 0x6E3 = 1763 (???)
{4,0x01,0x62,0x0D,0x78}, // LINE_LENGTH_A: 0xD78 = 3448
{4,0x01,0x5A,0x04,0x22}, // COARSE_INTEGRATION_TIME: 0x422 = 1058
{3,0x01,0x57,0x00,0},    // ANA_GAIN_GLOBAL_A: 0
{3,0x01,0x57,0x00,0},    // ANA_GAIN_GLOBAL_A: 0
{4,0x01,0x60,0x06,0xE3}, // FRM_LENGTH_A: 0x6E3
{4,0x01,0x62,0x0D,0x78}, // LINE_LENGTH_A: 0xD78 = 3448
{4,0x01,0x5A,0x04,0x22}, // COARSE_INTEGRATION_TIME: 0x422 = 1058
{3,0x01,0x00,0x01,0},    // GO!

{3,0x01,0x57,0x00,0},    // ANA_GAIN_GLOBAL_A: 0

{4,0x01,0x60,0x06,0xE3}, // FRM_LENGTH_A: 0x6E3
{4,0x01,0x62,0x0D,0x78}, // LINE_LENGTH_A: 0xD78 = 3448
{4,0x01,0x5A,0x04,0x21}, // COARSE_INTEGRATION_TIME: 0x421 = 1057
{3,0x01,0x57,0x00,0},    // ANA_GAIN_GLOBAL_A: 0

{4,0x01,0x60,0x06,0xE3}, // FRM_LENGTH_A: 0x6E3
{4,0x01,0x62,0x0D,0x78}, // LINE_LENGTH_A: 0xD78 = 3448
{4,0x01,0x5A,0x03,0xE2}, // COARSE_INTEGRATION_TIME
{3,0x01,0x57,0x00,0},    // ANA_GAIN_GLOBAL_A: 0

{4,0x01,0x60,0x06,0xE3}, // FRM_LENGTH_A: 0x6E3
{4,0x01,0x62,0x0D,0x78}, // LINE_LENGTH_A: 0xD78 = 3448
{4,0x01,0x5A,0x03,0xCD}, // COARSE_INTEGRATION_TIME
{3,0x01,0x57,0x00,0},    // ANA_GAIN_GLOBAL_A: 0

{4,0x01,0x60,0x06,0xE3}, // FRM_LENGTH_A: 0x6E3
{4,0x01,0x62,0x0D,0x78}, // LINE_LENGTH_A: 0xD78 = 3448
{4,0x01,0x5A,0x03,0xBA}, // COARSE_INTEGRATION_TIME
{3,0x01,0x57,0x00,0}     // ANA_GAIN_GLOBAL_A: 0
};
