#pragma once

#include <stdint.h>

enum SfxId : uint8_t {
  SFX_EAT_PLANT = 0,
  SFX_EAT_HERB,
  SFX_EAT_CARN,
  SFX_EAT_GARBAGE,
  SFX_EAT_SPORE,
  SFX_COUNT
};

/** Init amp, ES8311, I2S, and start the audio playback task. */
void sfx_begin();

/**
 * Queue a one-shot SFX (non-blocking).
 * Same-ID requests within cooldown_ms are dropped (used for predation nibbles).
 * No-op while muted.
 */
void sfx_play(SfxId id, uint32_t cooldown_ms = 0);

/** Poll BOOT button; toggles mute on press. Call from loop(). */
void sfx_poll();

/** True when eat SFX / amp are enabled. */
bool sfx_enabled();
