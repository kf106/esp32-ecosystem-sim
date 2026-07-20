#pragma once

/**
 * Pin definitions for the LCDwiki / Freenove 2.8" ESP32-S3 ES3N28P
 * (cheap obsidian display). Audio pins match Freenove FNK0104AB demos.
 */

// Audio (ES8311 codec + FM8002E amp)
// DOUT/DIN follow Freenove working music/echo sketches (not the wiki label swap).
#define BOARD_AUDIO_PIN_EN      1   // Active low
#define BOARD_AUDIO_PIN_MCK     4
#define BOARD_AUDIO_PIN_BCK     5
#define BOARD_AUDIO_PIN_WS      7
#define BOARD_AUDIO_PIN_DOUT    8   // ESP32 → codec SDIN
#define BOARD_AUDIO_PIN_DIN     6   // codec SDOUT → ESP32

#define BOARD_I2C_PIN_SDA       16
#define BOARD_I2C_PIN_SCL       15
#define BOARD_ES8311_ADDR       0x18

// BOOT button (also download-mode); active low when pressed
#define BOARD_BOOT_BUTTON_PIN   0
