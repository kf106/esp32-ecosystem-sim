#include "audio_sfx.h"

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <math.h>

#include "board_pins.h"
#include "sfx_data.h"

static constexpr i2s_port_t kI2sPort = I2S_NUM_0;
static constexpr int kQueueDepth = 2;
static constexpr uint32_t kGlobalCooldownMs = 450;

static QueueHandle_t s_sfxQueue = nullptr;
static uint32_t s_lastPlayMs[SFX_COUNT] = {0};
static uint32_t s_lastAnyPlayMs = 0;
static bool s_ready = false;
static volatile bool s_enabled = true;
static volatile int8_t s_chirpReq = 0; // +1 = unmute chirp, -1 = mute chirp
static bool s_bootWasDown = false;
static uint32_t s_bootEdgeMs = 0;


struct SfxClip {
  const int16_t *data;
  size_t len;
};

static const SfxClip kClips[SFX_COUNT] = {
  {sfx_eat_plant, sfx_eat_plant_len},
  {sfx_eat_herb, sfx_eat_herb_len},
  {sfx_eat_carn, sfx_eat_carn_len},
  {sfx_eat_garbage, sfx_eat_garbage_len},
  {sfx_eat_spore, sfx_eat_spore_len},
};

static bool es8311_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BOARD_ES8311_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static int es8311_read(uint8_t reg) {
  Wire.beginTransmission(BOARD_ES8311_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)BOARD_ES8311_ADDR, 1) != 1) return -1;
  return Wire.read();
}

/** DAC-only bring-up for 16 kHz with MCLK = 256 * fs (4.096 MHz). */
static bool es8311_init_dac() {
  // Reset
  if (!es8311_write(0x00, 0x1F)) return false;
  delay(20);
  es8311_write(0x00, 0x00);
  delay(5);

  // Clock manager basics, then slave + clocks from MCLK pin
  es8311_write(0x01, 0x30);
  es8311_write(0x02, 0x00);
  es8311_write(0x03, 0x10);
  es8311_write(0x04, 0x10);
  es8311_write(0x05, 0x00);
  es8311_write(0x0B, 0x00);
  es8311_write(0x0C, 0x00);
  es8311_write(0x10, 0x1F);
  es8311_write(0x11, 0x7F);
  es8311_write(0x00, 0x80); // CSM on, slave (bit6=0)
  es8311_write(0x01, 0x3F); // MCLK sourced from pin, all clocks on

  // Coeff for mclk=4096000, rate=16000:
  // pre_div=1, pre_multi=1, adc_div=1, dac_div=1, fs_mode=0,
  // lrck=0x00FF, bclk_div=4, osr=0x10
  es8311_write(0x02, 0x00);                         // pre_div-1=0, pre_multi=1
  es8311_write(0x05, 0x00);                         // adc/dac div = 1
  es8311_write(0x03, 0x10);                         // adc OSR
  es8311_write(0x04, 0x10);                         // dac OSR
  es8311_write(0x07, 0x00);                         // LRCK high
  es8311_write(0x08, 0xFF);                         // LRCK low (256fs-1)
  es8311_write(0x06, 0x03);                         // BCLK div = 4

  // I2S Philips, 16-bit on DAC SDP
  es8311_write(0x09, 0x0C); // 16-bit length bits + normal I2S
  es8311_write(0x0A, 0x0C);

  // Power / analog
  es8311_write(0x0D, 0x01);
  es8311_write(0x0E, 0x02);
  es8311_write(0x12, 0x00);
  es8311_write(0x13, 0x10);
  es8311_write(0x1B, 0x0A);
  es8311_write(0x1C, 0x6A);
  es8311_write(0x37, 0x48);
  es8311_write(0x44, 0x08);
  es8311_write(0x45, 0x00);
  es8311_write(0x14, 0x1A); // enable DAC analog
  es8311_write(0x31, 0x00); // unmute
  es8311_write(0x32, 0xB0); // quieter desk-toy volume (was 0xFF)

  int chip = es8311_read(0x00);
  Serial.printf("sfx: ES8311 REG00=0x%02X\n", chip);
  return chip >= 0;
}

static bool i2s_init_tx() {
  // Mono left slot — ES8311 is a mono DAC and Freenove demos use LEFT
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SFX_SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  esp_err_t err = i2s_driver_install(kI2sPort, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("sfx: i2s_driver_install failed %d\n", (int)err);
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.mck_io_num = BOARD_AUDIO_PIN_MCK;
  pins.bck_io_num = BOARD_AUDIO_PIN_BCK;
  pins.ws_io_num = BOARD_AUDIO_PIN_WS;
  pins.data_out_num = BOARD_AUDIO_PIN_DOUT;
  pins.data_in_num = BOARD_AUDIO_PIN_DIN;

  err = i2s_set_pin(kI2sPort, &pins);
  if (err != ESP_OK) {
    Serial.printf("sfx: i2s_set_pin failed %d\n", (int)err);
    return false;
  }

  // Start clocks (MCLK) before codec register programming finishes settling
  i2s_zero_dma_buffer(kI2sPort);
  return true;
}

static void i2s_write_mono(const int16_t *samples, size_t count, bool from_progmem) {
  static int16_t buf[256];
  size_t i = 0;
  while (i < count) {
    size_t n = count - i;
    if (n > 256) n = 256;
    for (size_t s = 0; s < n; s++) {
      buf[s] = from_progmem ? (int16_t)pgm_read_word(&samples[i + s]) : samples[i + s];
    }
    size_t written = 0;
    i2s_write(kI2sPort, buf, n * sizeof(int16_t), &written, portMAX_DELAY);
    i += n;
  }
}

static void set_amp(bool on) {
  // Active-low enable
  digitalWrite(BOARD_AUDIO_PIN_EN, on ? LOW : HIGH);
}

static void play_clip(const SfxClip &clip) {
  if (!s_enabled) return;
  i2s_write_mono(clip.data, clip.len, true);
}

static void play_boot_beep() {
  // Soft short confirmation (~120 ms, quieter)
  const int n = SFX_SAMPLE_RATE / 8;
  static int16_t tone[SFX_SAMPLE_RATE / 8];
  for (int i = 0; i < n; i++) {
    float env = 1.0f;
    if (i < 150) env = i / 150.0f;
    if (i > n - 300) env = (n - i) / 300.0f;
    tone[i] = (int16_t)(sinf(2.0f * 3.1415926f * 660.0f * i / SFX_SAMPLE_RATE) * 7000.0f * env);
  }
  i2s_write_mono(tone, n, false);
}

static void play_mute_chirp(bool turning_on) {
  const float hz = turning_on ? 880.0f : 330.0f;
  const int n = SFX_SAMPLE_RATE / 16; // ~60 ms
  static int16_t tone[SFX_SAMPLE_RATE / 16];
  for (int i = 0; i < n; i++) {
    float env = 1.0f;
    if (i < 80) env = i / 80.0f;
    if (i > n - 120) env = (n - i) / 120.0f;
    tone[i] = (int16_t)(sinf(2.0f * 3.1415926f * hz * i / SFX_SAMPLE_RATE) * 6000.0f * env);
  }
  set_amp(true);
  i2s_write_mono(tone, n, false);
  if (!turning_on) {
    set_amp(false);
  }
}

static void audioTask(void * /*arg*/) {
  delay(50);
  if (s_enabled) {
    play_boot_beep();
    Serial.println("sfx: boot beep done");
  }

  SfxId id;
  for (;;) {
    if (s_chirpReq != 0) {
      int8_t req = s_chirpReq;
      s_chirpReq = 0;
      play_mute_chirp(req > 0);
    }

    if (xQueueReceive(s_sfxQueue, &id, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (!s_enabled) continue;
      if (id < SFX_COUNT) {
        play_clip(kClips[id]);
      }
    }
  }
}

void sfx_begin() {
  pinMode(BOARD_AUDIO_PIN_EN, OUTPUT);
  set_amp(true);

  pinMode(BOARD_BOOT_BUTTON_PIN, INPUT_PULLUP);
  s_bootWasDown = digitalRead(BOARD_BOOT_BUTTON_PIN) == LOW;

  Wire.begin(BOARD_I2C_PIN_SDA, BOARD_I2C_PIN_SCL, 400000);
  delay(10);

  // I2S first so MCLK is running before ES8311 clock config
  if (!i2s_init_tx()) {
    Serial.println("sfx: I2S init failed");
    return;
  }
  delay(20);

  if (!es8311_init_dac()) {
    Serial.println("sfx: ES8311 init failed (no ACK on I2C 0x18?)");
  }

  s_sfxQueue = xQueueCreate(kQueueDepth, sizeof(SfxId));
  if (!s_sfxQueue) {
    Serial.println("sfx: queue create failed");
    return;
  }

  xTaskCreatePinnedToCore(audioTask, "sfx", 6144, nullptr, 4, nullptr, 0);
  s_ready = true;
  Serial.println("sfx: ready (BOOT toggles mute)");
}

void sfx_play(SfxId id, uint32_t cooldown_ms) {
  if (!s_ready || !s_enabled || !s_sfxQueue || id >= SFX_COUNT) return;

  uint32_t now = millis();
  if ((now - s_lastAnyPlayMs) < kGlobalCooldownMs) return;
  if (cooldown_ms > 0) {
    if ((now - s_lastPlayMs[id]) < cooldown_ms) return;
  }
  s_lastPlayMs[id] = now;
  s_lastAnyPlayMs = now;

  xQueueSend(s_sfxQueue, &id, 0);
}

void sfx_poll() {
  if (!s_ready) return;

  const bool down = digitalRead(BOARD_BOOT_BUTTON_PIN) == LOW;
  const uint32_t now = millis();

  // Debounced press edge: toggle mute
  if (down && !s_bootWasDown && (now - s_bootEdgeMs) > 40) {
    s_bootEdgeMs = now;
    const bool nowOn = !s_enabled;
    s_enabled = nowOn;

    if (!nowOn && s_sfxQueue) {
      SfxId dump;
      while (xQueueReceive(s_sfxQueue, &dump, 0) == pdTRUE) {}
    }

    s_chirpReq = nowOn ? 1 : -1;
    Serial.printf("sfx: %s\n", nowOn ? "on" : "muted");
  }
  if (down != s_bootWasDown) {
    s_bootEdgeMs = now;
  }
  s_bootWasDown = down;
}

bool sfx_enabled() {
  return s_enabled;
}
