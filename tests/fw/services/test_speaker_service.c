/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

#include "clar.h"

#include "pbl/services/speaker/speaker_service.h"

#include <pbl/drivers/audio.h>
#include <pbl/drivers/rtc.h>

#include <string.h>

#include "stubs_logging.h"
#include "stubs_passert.h"
#include "stubs_analytics.h"
#include "stubs_rtc.h"
#include "stubs_do_not_disturb.h"
#include "fake_mutex.h"
#include "fake_pbl_malloc.h"
#include "fake_system_task.h"
#include "fake_events.h"
#include "fake_pebble_tasks.h"

// Alerts preferences: speaker unmuted, no volume cap
bool alerts_preferences_get_speaker_muted(void) { return false; }
uint8_t alerts_preferences_get_speaker_volume(void) { return 100; }
bool alerts_preferences_dnd_get_mute_speaker(void) { return false; }

// ---------------------------------------------------------------------------
// Fake audio driver, modelled on the sf32lb52 one: a 4096 byte buffer that
// keeps what fits and silently drops the rest, emptied 1024 bytes at a time by
// the DMA, which plays silence for whatever it is short of. It counts what the
// service hands to the hardware so the tests can tell how much audio would
// actually have been played before the service powered the speaker down.

#define DRIVER_CAPACITY_BYTES 4096
#define DRIVER_PERIOD_BYTES 1024
#define CAPTURE_SAMPLES 65536

static AudioTransCB s_trans_cb;
static int s_start_count;
static int s_stop_count;
static uint32_t s_samples_written;
static uint32_t s_nonzero_samples;
static uint32_t s_queued_bytes;
static uint32_t s_dropped_bytes;
static uint32_t s_consumed_bytes;
static uint32_t s_consumed_bytes_at_stop;
static int16_t s_captured[CAPTURE_SAMPLES];

void audio_init(AudioDevice *device) {}

void audio_start(AudioDevice *device, AudioTransCB cb) {
  s_trans_cb = cb;
  s_start_count++;
  s_queued_bytes = 0;
}

uint32_t audio_write(AudioDevice *device, void *buf, uint32_t size) {
  const uint32_t free_bytes = DRIVER_CAPACITY_BYTES - s_queued_bytes;
  const uint32_t accepted = (size > free_bytes) ? free_bytes : size;
  s_dropped_bytes += size - accepted;
  s_queued_bytes += accepted;

  const int16_t *samples = buf;
  const uint32_t num_samples = accepted / sizeof(int16_t);
  for (uint32_t i = 0; i < num_samples; i++) {
    if (samples[i] != 0) {
      s_nonzero_samples++;
    }
    if (s_samples_written < CAPTURE_SAMPLES) {
      s_captured[s_samples_written] = samples[i];
    }
    s_samples_written++;
  }
  return DRIVER_CAPACITY_BYTES - s_queued_bytes;
}

void audio_set_volume(AudioDevice *device, int volume) {}

void audio_stop(AudioDevice *device) {
  s_stop_count++;
  s_trans_cb = NULL;
  s_consumed_bytes_at_stop = s_consumed_bytes;
}

// ---------------------------------------------------------------------------

#define SAMPLE_RATE 16000
// Must match SPEAKER_PIPELINE_DRAIN_SAMPLES in speaker_service.c
#define DRAIN_SAMPLES ((SAMPLE_RATE * 80) / 1000)

static void prv_dma_consume(void) {
  const uint32_t n = (s_queued_bytes < DRIVER_PERIOD_BYTES) ? s_queued_bytes : DRIVER_PERIOD_BYTES;
  s_queued_bytes -= n;
  s_consumed_bytes += n;
}

static void prv_request_refill(void) {
  if (s_trans_cb) {
    uint32_t free_size = DRIVER_CAPACITY_BYTES - s_queued_bytes;
    s_trans_cb(&free_size);
  }
  fake_system_task_callbacks_invoke_pending();
}

// Simulate the driver requesting more data until playback stops on its own.
static void prv_pump_until_idle(void) {
  for (int i = 0; i < 100 && s_trans_cb; i++) {
    prv_dma_consume();
    prv_request_refill();
  }
}

void test_speaker_service__initialize(void) {
  fake_system_task_callbacks_cleanup();
  s_trans_cb = NULL;
  s_start_count = 0;
  s_stop_count = 0;
  s_samples_written = 0;
  s_nonzero_samples = 0;
  s_queued_bytes = 0;
  s_dropped_bytes = 0;
  s_consumed_bytes = 0;
  s_consumed_bytes_at_stop = 0;
  speaker_service_init();
}

void test_speaker_service__cleanup(void) {
  speaker_service_stop();
  fake_system_task_callbacks_cleanup();
}

// A tone shorter than the driver's DMA pipeline (~64 ms) must still be
// generated in full and followed by enough padding silence that the queued
// samples play out before the service stops the hardware.
void test_speaker_service__short_tone_drains_pipeline_before_stop(void) {
  const uint16_t duration_ms = 25;
  const uint32_t tone_samples = (SAMPLE_RATE * duration_ms) / 1000;

  cl_assert(speaker_service_play_tone(1000, duration_ms, 0 /* sine */,
                                      0 /* full velocity */,
                                      SpeakerPriorityApp, 80));
  cl_assert_equal_i(speaker_service_get_state(), SpeakerStatePlaying);

  prv_pump_until_idle();

  cl_assert_equal_i(speaker_service_get_state(), SpeakerStateIdle);
  cl_assert_equal_i(s_start_count, 1);
  cl_assert_equal_i(s_stop_count, 1);
  // The full tone reached the driver (1 kHz sine: allow for zero crossings)
  cl_assert(s_nonzero_samples > tone_samples / 2);
  // ...followed by at least a pipeline depth of padding silence
  cl_assert(s_samples_written >= tone_samples + DRAIN_SAMPLES);
}

// While only padding silence remains, a new same-priority sound must be able
// to start instead of being rejected (rapid re-triggers, e.g. a metronome).
void test_speaker_service__same_priority_preempts_during_drain(void) {
  cl_assert(speaker_service_play_tone(1000, 25, 0, 0, SpeakerPriorityApp, 80));

  prv_request_refill();
  cl_assert_equal_i(speaker_service_get_state(), SpeakerStateDraining);

  cl_assert(speaker_service_play_tone(2000, 25, 0, 0, SpeakerPriorityApp, 80));
  cl_assert_equal_i(speaker_service_get_state(), SpeakerStatePlaying);

  prv_pump_until_idle();
  cl_assert_equal_i(speaker_service_get_state(), SpeakerStateIdle);
}

// A sound still generating real samples must NOT be preemptable by the same
// priority (unchanged behavior).
void test_speaker_service__same_priority_cannot_preempt_while_playing(void) {
  cl_assert(speaker_service_play_tone(1000, 500, 0, 0, SpeakerPriorityApp, 80));
  cl_assert_equal_i(speaker_service_get_state(), SpeakerStatePlaying);

  cl_assert(!speaker_service_play_tone(2000, 25, 0, 0, SpeakerPriorityApp, 80));
}

// ---------------------------------------------------------------------------
// Streams

#define STREAM_BYTES 16000

static int8_t prv_stream_byte(uint32_t i) {
  return (int8_t)(1 + (i % 120));
}

static uint32_t s_stream_sent;

static void prv_stream_feed(uint32_t max_bytes) {
  uint8_t chunk[512];
  while (max_bytes > 0 && s_stream_sent < STREAM_BYTES) {
    uint32_t n = STREAM_BYTES - s_stream_sent;
    if (n > sizeof(chunk)) {
      n = sizeof(chunk);
    }
    if (n > max_bytes) {
      n = max_bytes;
    }
    for (uint32_t i = 0; i < n; i++) {
      chunk[i] = (uint8_t)prv_stream_byte(s_stream_sent + i);
    }
    const uint32_t written = speaker_service_stream_write(chunk, n);
    s_stream_sent += written;
    max_bytes -= written;
    if (written < n) {
      return;
    }
  }
}

// Every sample the stream took must reach the driver, in order, with no
// silence spliced in between.
static void prv_assert_stream_played_intact(void) {
  cl_assert_equal_i(s_dropped_bytes, 0);
  cl_assert(s_samples_written <= CAPTURE_SAMPLES);

  uint32_t first = 0;
  while (first < s_samples_written && s_captured[first] == 0) {
    first++;
  }
  for (uint32_t i = 0; i < STREAM_BYTES; i++) {
    cl_assert_equal_i(s_captured[first + i], (int16_t)prv_stream_byte(i) << 8);
  }
}

void test_speaker_service__stream_never_overfills_driver(void) {
  s_stream_sent = 0;
  cl_assert(speaker_service_stream_open(SpeakerPriorityApp, 80, SpeakerPcmFormat_16kHz_8bit));

  for (int i = 0; i < 1000 && s_stream_sent < STREAM_BYTES; i++) {
    prv_stream_feed(STREAM_BYTES);
    prv_dma_consume();
    // The driver's ISR may queue a second refill before the first has run.
    prv_request_refill();
    prv_request_refill();
  }
  speaker_service_stream_close();
  prv_pump_until_idle();

  cl_assert_equal_i(speaker_service_get_state(), SpeakerStateIdle);
  prv_assert_stream_played_intact();
}

void test_speaker_service__stream_writes_no_silence_while_momentarily_empty(void) {
  s_stream_sent = 0;
  cl_assert(speaker_service_stream_open(SpeakerPriorityApp, 80, SpeakerPcmFormat_16kHz_8bit));

  for (int i = 0; i < 1000 && s_stream_sent < STREAM_BYTES; i++) {
    if (i % 3 == 0) {
      prv_stream_feed(700);
    }
    prv_dma_consume();
    prv_request_refill();
  }
  speaker_service_stream_close();
  prv_pump_until_idle();

  cl_assert_equal_i(speaker_service_get_state(), SpeakerStateIdle);
  prv_assert_stream_played_intact();
}

// Closing a stream the driver has already taken everything from must still let
// the driver's buffer play out before the speaker is powered down.
void test_speaker_service__stream_close_plays_out_driver_buffer(void) {
  s_stream_sent = 0;
  cl_assert(speaker_service_stream_open(SpeakerPriorityApp, 80, SpeakerPcmFormat_16kHz_8bit));

  prv_stream_feed(DRIVER_CAPACITY_BYTES / sizeof(int16_t));
  prv_request_refill();
  cl_assert_equal_i(s_queued_bytes, DRIVER_CAPACITY_BYTES);

  speaker_service_stream_close();
  cl_assert_equal_i(speaker_service_get_state(), SpeakerStateDraining);
  cl_assert_equal_i(s_stop_count, 0);

  prv_pump_until_idle();

  cl_assert_equal_i(speaker_service_get_state(), SpeakerStateIdle);
  cl_assert_equal_i(s_dropped_bytes, 0);
  // Both DMA halves must have moved past the last real sample.
  cl_assert(s_consumed_bytes_at_stop >= DRIVER_CAPACITY_BYTES + 2 * DRIVER_PERIOD_BYTES);
}
