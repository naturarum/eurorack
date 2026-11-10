// Copyright 2017 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Generates the delay time map.
//
// Modified in 2026 by naturarum: TIME sets the delay time and DENSITY picks a
// pattern of taps. See delay_controller.h.

#include "beads/dsp/delay_controller.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

namespace beads {

using namespace std;
using namespace stmlib;

const int kMinDelay = 64 + 4;

// TIME without a clock: 5 ms to 2.56 s on the first 85% of the knob, then up
// to the whole buffer (4 s to 32 s depending on the quality and on mono).
const float kMinDelayTime = 0.005f;
const float kKneeTime = 0.85f;
const float kKneeOctaves = 9.0f;

// TIME with a clock: delay time as a fraction of the clock period. Noon is
// one period.
const int kNumTimeRatios = 15;
const float kTimeRatio[kNumTimeRatios] = {
  1.0f / 8.0f,
  1.0f / 6.0f,
  1.0f / 4.0f,
  1.0f / 3.0f,
  1.0f / 2.0f,
  2.0f / 3.0f,
  3.0f / 4.0f,
  1.0f,
  4.0f / 3.0f,
  3.0f / 2.0f,
  2.0f,
  3.0f,
  4.0f,
  6.0f,
  8.0f,
};

// DENSITY: patterns of taps, from left to right. The main tap repeats at the
// delay time. The other taps play every repeat of the main tap once, at a
// fraction of the delay time, and do not feed back.
struct PatternTap {
  float ratio;
  float gain;
  float pan;
  float feedback;
};

struct DelayPattern {
  float main_pan;
  float main_feedback;
  // The extra taps' gain and feedback follow the FEEDBACK knob.
  bool ping_pong;
  PatternTap extra[DelayController::NUM_PATTERN_TAPS - 1];
};

const int kNumPatterns = 5;
const DelayPattern kPatterns[kNumPatterns] = {
  // Single.
  { 0.0f, 1.0f, false, {
      { 1.0f, 0.0f, 0.0f, 0.0f },
      { 1.0f, 0.0f, 0.0f, 0.0f },
      { 1.0f, 0.0f, 0.0f, 0.0f } } },
  // Ping-pong. The main tap plays the odd repeats on the left. A tap at twice
  // the time plays the even ones on the right, and it alone feeds back: the
  // repeats alternate even with a mono buffer.
  { -1.0f, 0.0f, true, {
      { 2.0f, 1.0f, 1.0f, 1.0f },
      { 1.0f, 0.0f, 0.0f, 0.0f },
      { 1.0f, 0.0f, 0.0f, 0.0f } } },
  // Dotted: an extra tap at 3/4 of the time.
  { 0.0f, 1.0f, false, {
      { 0.75f, 0.7f, -0.5f, 0.0f },
      { 1.0f, 0.0f, 0.0f, 0.0f },
      { 1.0f, 0.0f, 0.0f, 0.0f } } },
  // Triplet: extra taps at 1/3 and 2/3 of the time.
  { 0.0f, 1.0f, false, {
      { 1.0f / 3.0f, 0.55f, -0.5f, 0.0f },
      { 2.0f / 3.0f, 0.7f, 0.5f, 0.0f },
      { 1.0f, 0.0f, 0.0f, 0.0f } } },
  // Four taps: extra taps at 1/4, 1/2 and 3/4 of the time.
  { 0.0f, 1.0f, false, {
      { 0.25f, 0.45f, -0.6f, 0.0f },
      { 0.5f, 0.6f, 0.6f, 0.0f },
      { 0.75f, 0.75f, -0.3f, 0.0f } } },
};

// Width of the fades between patterns, in units of a pattern's zone. The
// extra taps fade out on both sides of a border, so they can change ratio.
const float kPatternFade = 0.12f;

void DelayController::Init(float sample_rate, int max_delay) {
  max_delay_ = max_delay;
  sample_rate_ = sample_rate;
  knee_delay_ = kMinDelayTime * sample_rate * \
      SemitonesToRatio(12.0f * kKneeOctaves);
  // Octaves from the knee to the whole buffer. logf would pull errno
  // handling from the C library, and a rough log2 is enough here.
  float ratio = float(max_delay) / knee_delay_;
  float octaves = 0.0f;
  if (ratio > 1.0f) {
    while (ratio >= 2.0f) {
      ratio *= 0.5f;
      octaves += 1.0f;
    }
    const float x = ratio - 1.0f;
    octaves += x * (1.3466f - 0.3466f * x);
  }
  top_semitones_ = 12.0f * octaves;

  clock_.Init(sample_rate);
  
  for (int i = 0; i < NUM_SLOTS; ++i) {
    numerator_quantizer_[i].Init(2, 0.05f, false);
    play_head_[i].Init();
    slave_play_head_[i].Init();
    slot_delay_[i] = sample_rate * 0.25f;
  }
  ratio_quantizer_.Init(kNumTimeRatios, 0.25f, false);
  current_slot_ = 0;
  slot_fade_ = 1.0f;
  slot_fade_increment_ = 0.0f;
  
  for (int i = 0; i < NUM_TAPS; ++i) {
    tap_envelope_[i].Init();
  }
  
  position_modulation_.Init(0.1f / sample_rate);
  shape_modulation_.Init(0.1f / sample_rate);
  
  const float kFilterCoefficient = 0.0025f;
  density_filter_.Init(kFilterCoefficient);
  slice_index_filter_.Init(kFilterCoefficient);

  time_lp_ = -1.0f;
  shape_ = 0.0f;
  linear_fm_ = 0.0f;
  density_ = 0.0f;
  
  freeze_ = false;
  seed_button_state_ = SEED_BUTTON_STATE_OFF;
}

float DelayController::FreeDelay(float time) const {
  if (time <= kKneeTime || top_semitones_ <= 0.0f) {
    return kMinDelayTime * sample_rate_ * SemitonesToRatio(
        12.0f * kKneeOctaves * time / kKneeTime);
  } else {
    // Exponential from the knee to the whole buffer.
    const float x = (time - kKneeTime) / (1.0f - kKneeTime);
    return knee_delay_ * SemitonesToRatio(top_semitones_ * x);
  }
}

void DelayController::UpdateSlots(float target_delay) {
  if (slot_fade_ < 1.0f) {
    slot_fade_ += slot_fade_increment_;
    if (slot_fade_ > 1.0f) {
      slot_fade_ = 1.0f;
    }
    return;
  }
  const float current = slot_delay_[current_slot_];
  const float threshold = max(current * 0.004f, 2.0f);
  if (fabsf(target_delay - current) > threshold) {
    current_slot_ = 1 - current_slot_;
    slot_delay_[current_slot_] = target_delay;
    slot_fade_ = 0.0f;
    // Fade over half the delay time, between 20 ms and 120 ms.
    float duration = 0.5f * target_delay;
    CONSTRAIN(duration, 0.02f * sample_rate_, 0.12f * sample_rate_);
    slot_fade_increment_ = 1.0f / duration;
    for (int i = 0; i < NUM_PATTERN_TAPS; ++i) {
      tap_envelope_[current_slot_ * NUM_PATTERN_TAPS + i].Init();
    }
    play_head_[current_slot_].Init();
    slave_play_head_[current_slot_].Init();
  }
}

void DelayController::Render(
    SeedButtonState seed_button_state,
    const stmlib::GateFlags* external_input,
    const bool* freeze,
    float density,
    float linear_fm,
    float exponential_fm,
    const RandomizedParameter& position,
    const RandomizedParameter& shape,
    float feedback,
    Output* out,
    size_t size) {
  // The DENSITY CV moves through the patterns like the knob.
  density += linear_fm + exponential_fm;
  if (seed_button_state == SEED_BUTTON_STATE_LOCKED && !external_input) {
    RenderInternal<false>(
        seed_button_state,
        external_input,
        freeze,
        density,
        position,
        shape, feedback, out, size);
  } else {
    RenderInternal<true>(
        seed_button_state,
        external_input,
        freeze,
        density,
        position,
        shape, feedback, out, size);
  }
}

template<bool synchronized>
void DelayController::RenderInternal(
    SeedButtonState seed_button_state,
    const stmlib::GateFlags* external_input,
    const bool* freeze,
    float density,
    const RandomizedParameter& position,
    const RandomizedParameter& shape,
    float feedback,
    Output* out,
    size_t size) {
  ONE_POLE(linear_fm_, density, 0.05f);
  density = linear_fm_;
  CONSTRAIN(density, 0.0f, 1.0f);
  
  float time = 1.005f * position.value(
      &position_modulation_,
      size,
      1.0f,
      true);
  CONSTRAIN(time, 0.0f, 1.0f);
  if (time_lp_ < 0.0f) {
    time_lp_ = time;
  }
  ParameterInterpolator modulated_shape(
      &shape_,
      shape.value(&shape_modulation_, size, 1.0f, true) * 0.995f,
      size);
  ParameterInterpolator modulated_density(
      &density_,
      density_filter_.Process(density),
      size);
  
  bool tap = (seed_button_state == SEED_BUTTON_STATE_ON) && \
      (seed_button_state_ == SEED_BUTTON_STATE_OFF);
  seed_button_state_ = seed_button_state;
  
  // Ping-pong: repeat gain of the feedback path at small levels.
  float ping_pong_gain = 1.5f * feedback * feedback;
  CONSTRAIN(ping_pong_gain, 0.0f, 1.0f);
  
  const int ratio_index = synchronized ? ratio_quantizer_.Process(time) : 0;
  
  while (size--) {
    const float density = modulated_density.Next();
    float target_delay;
    
    if (synchronized) {
      // Record the interval between taps on the button or triggers
      // on the SEED input.
      clock_.Process(
          external_input && (*external_input & GATE_FLAG_RISING), tap);
      tap = false; // The tap is recorded only once!
      target_delay = clock_.latest_interval() * kTimeRatio[ratio_index];
    } else {
      ONE_POLE(time_lp_, time, 0.0005f);
      target_delay = FreeDelay(time_lp_);
    }
    CONSTRAIN(target_delay, float(kMinDelay), float(max_delay_));
    UpdateSlots(target_delay);
    
    const float slot_gain[NUM_SLOTS] = {
      current_slot_ == 0 ? slot_fade_ : 1.0f - slot_fade_,
      current_slot_ == 1 ? slot_fade_ : 1.0f - slot_fade_,
    };

    const float shape = modulated_shape.Next();
    
    if (*freeze) {
      // Frozen: loop slices of the buffer as long as the delay time.
      // DENSITY picks the slice.
      if (!freeze_) {
        for (int i = 0; i < NUM_SLOTS; ++i) {
          play_head_[i].Init();
          slave_play_head_[i].Init();
        }
      }
      
      float slice_index_smooth = slice_index_filter_.Process(density);
      
      for (int i = 0; i < NUM_SLOTS; ++i) {
        PlayHead* new_head = NULL;
        
        if (play_head_[i].done()) {
          new_head = &play_head_[i];
        } else if (play_head_[i].half_way()) {
          new_head = &slave_play_head_[i];
        }
        
        if (new_head) {
          const int slice_duration = int(slot_delay_[i]);
          const int num_slices = max(max_delay_ / slice_duration, 1);
          if (numerator_quantizer_[i].num_steps() != num_slices) {
            numerator_quantizer_[i].Init(
                num_slices,
                0.05f / 5.0f * float(num_slices),
                false);
          }
          
          const int slice_index = numerator_quantizer_[i].Process(
              slice_index_smooth);
          
          new_head->Start(
              slice_duration * slice_index + kMinDelay,
              slice_duration,
              1.0f);
        }
        DelayTap* taps = &out->tap[i * NUM_PATTERN_TAPS];
        play_head_[i].Render(&taps[0], shape, false);
        slave_play_head_[i].Render(&taps[1], shape, true);
        taps[0].gain *= slot_gain[i];
        taps[1].gain *= slot_gain[i];
        for (int j = 2; j < NUM_PATTERN_TAPS; ++j) {
          taps[j].delay = 0;
          taps[j].gain = 0.0f;
          taps[j].pan = 0.0f;
          taps[j].feedback = 0.0f;
        }
      }
      out->gate = play_head_[current_slot_].phase() < 0.5f;
    } else {
      // Reset all envelopes when unfreezing.
      if (freeze_) {
        for (int i = 0; i < NUM_TAPS; ++i) {
          tap_envelope_[i].Init();
        }
      }
      
      // Pattern, with fades on both sides of each border.
      float position = density * float(kNumPatterns);
      MAKE_INTEGRAL_FRACTIONAL(position);
      if (position_integral >= kNumPatterns) {
        position_integral = kNumPatterns - 1;
        position_fractional = 1.0f;
      }
      float presence = 1.0f;
      if (position_integral > 0) {
        presence = min(presence, position_fractional / kPatternFade);
      }
      if (position_integral < kNumPatterns - 1) {
        presence = min(presence, (1.0f - position_fractional) / kPatternFade);
      }
      CONSTRAIN(presence, 0.0f, 1.0f);
      const DelayPattern& pattern = kPatterns[position_integral];
      const float extra_scale = pattern.ping_pong ? ping_pong_gain : 1.0f;
      
      for (int slot = 0; slot < NUM_SLOTS; ++slot) {
        const float slot_delay = slot_delay_[slot];
        const float gain = slot_gain[slot];
        for (int j = 0; j < NUM_PATTERN_TAPS; ++j) {
          const int i = slot * NUM_PATTERN_TAPS + j;
          DelayTap* t = &out->tap[i];
          float ratio, tap_gain, pan, tap_feedback;
          if (j == 0) {
            ratio = 1.0f;
            tap_gain = 1.0f;
            pan = pattern.main_pan * presence;
            tap_feedback = 1.0f + (pattern.main_feedback - 1.0f) * presence;
          } else {
            const PatternTap& extra = pattern.extra[j - 1];
            ratio = extra.ratio;
            tap_gain = extra.gain * presence * extra_scale;
            pan = extra.pan;
            tap_feedback = extra.feedback * presence * extra_scale;
          }
          if (gain == 0.0f || (tap_gain == 0.0f && tap_feedback == 0.0f)) {
            t->delay = 0;
            t->gain = 0.0f;
            t->pan = 0.0f;
            t->feedback = 0.0f;
            continue;
          }
          float delay_time = slot_delay * ratio;
          CONSTRAIN(delay_time, kMinDelay, max_delay_);
          tap_envelope_[i].set_phase_increment(1.0f / delay_time);
          const float envelope = tap_envelope_[i].Render(shape) * gain;
          t->delay = int32_t(delay_time * 1024.0f);
          t->gain = tap_gain * envelope;
          t->pan = pan;
          t->feedback = tap_feedback * envelope;
        }
      }
      out->gate = tap_envelope_[current_slot_ * NUM_PATTERN_TAPS].phase() < \
          0.5f;
    }

    freeze_ = *freeze;
    
    if (external_input) {
      ++external_input;
    }
    ++freeze;
    ++out;
  }
}

}  // namespace beads
