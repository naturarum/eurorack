// Copyright 2026 naturarum.
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
// Reverb with a continuous character control, from room through hall to
// spring.
//
// The tank is an 8-line feedback delay network. Its mixing matrix is built
// from three levels of 2x2 rotations, as in A. Coppola, "Fast parametric
// matrices for lossless feedback delay networks" (DAFx 2026): with all angles
// at pi/4 every line feeds every other one equally, at 0 the lines ring on
// their own. The angles are the diffusion control.
//
// Lines 0 to 3 run through cascades of stretched allpass filters, the
// dispersive element of V. Valimaki, J. Parker and J. Abel, "Parametric spring
// reverberation effect" (JAES 2010). With a coefficient of 0 a stretched
// allpass is a plain delay, so the spring fades in without any switch. Towards
// the spring end, lines 4 to 7 fade out and the top matrix level decouples
// the two groups of lines.
//
// Early reflections are a sparse set of taps on the pre-delay line, which
// also feeds the network through four allpass diffusers.

#ifndef BEADS_DSP_FX_MORPH_REVERB_H_
#define BEADS_DSP_FX_MORPH_REVERB_H_

#include "stmlib/stmlib.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"

#include "beads/dsp/frame.h"

namespace beads {

// Controls of the shift layer, all in [0, 1].
struct ReverbParameters {
  float character = 0.5f;  // 0: room, 0.5: hall, 1: spring.
  float size = 0.5f;
  float decay = 0.5f;
  float tone = 0.55f;
  float diffusion = 0.8f;
};

const size_t kReverbNumLines = 8;
const size_t kReverbNumSpringLines = 4;
const size_t kReverbNumSpringStages = 24;
const int32_t kReverbMaxSpringStretch = 7;
const size_t kReverbNumEarlyTaps = 6;
const size_t kReverbNumDiffusers = 4;
const size_t kReverbMaxBlockSize = 24;

// The large pool is the buffer passed to Init (SRAM4 on the module). The
// small pool is a member array, which ends up in DTCM with the processor.
const uint32_t kReverbLargePoolSize = 32768;
const uint32_t kReverbSmallPoolSize = 16384;

// Delay lines are read in blocks, so they must never be shorter than a block.
const float kReverbMinDelay = float(kReverbMaxBlockSize + 2);
const float kReverbMaxSizeScale = 1.6f;
const float kReverbMaxEarlyMs = 100.0f;

// One end point of the character control.
struct ReverbCharacter {
  float line_ms[kReverbNumLines];
  float pre_delay_ms;
  float early_level;
  float early_span_ms;
  float decay_scale;
  float coupling[3];  // Scales the matrix angles, from the finest level up.
  float second_group;  // Level of lines 4 to 7.
  float dispersion;  // Stretched allpass coefficient on lines 0 to 3.
  float modulation_ms;
  float modulation_hz;
  float input_hp_hz;
  float input_lp_hz;
  float diffuser;
  float output_gain;
};

const ReverbCharacter kReverbRoom = {
  { 8.0f, 11.5f, 15.5f, 22.0f, 9.5f, 13.5f, 18.5f, 25.5f },
  2.0f, 0.6f, 35.0f, 0.35f,
  { 1.0f, 1.0f, 1.0f }, 1.0f, 0.0f,
  0.08f, 0.35f, 90.0f, 16000.0f, 0.9f, 0.8f
};

const ReverbCharacter kReverbHall = {
  { 31.0f, 47.0f, 61.0f, 83.0f, 37.0f, 53.0f, 73.0f, 97.0f },
  28.0f, 0.3f, 70.0f, 1.0f,
  { 1.0f, 1.0f, 1.0f }, 1.0f, 0.0f,
  0.45f, 0.5f, 25.0f, 14000.0f, 1.0f, 1.0f
};

// Lines 4 to 7 keep their hall lengths: they are silent at this end.
const ReverbCharacter kReverbSpring = {
  { 55.0f, 63.0f, 71.0f, 79.0f, 37.0f, 53.0f, 73.0f, 97.0f },
  18.0f, 0.0f, 40.0f, 0.9f,
  { 0.35f, 0.35f, 0.0f }, 0.0f, 0.7f,
  0.15f, 2.5f, 160.0f, 3200.0f, 0.25f, 1.45f
};

class MorphReverb {
 public:
  MorphReverb() { }
  ~MorphReverb() { }

  void Init(uint16_t* buffer, float sample_rate) {
    pool_[0] = buffer;
    pool_[1] = small_pool_;
    mix_ = 0.0f;
    dry_ = 0.0f;
    wet_ = 0.0f;
    send_ = 0.0f;
    Reset(sample_rate);
  }

  // Also clears the tank: delay lengths depend on the sample rate.
  void Reset(float sample_rate) {
    sample_rate_ = sample_rate;
    samples_per_ms_ = sample_rate * 0.001f;
    // The dispersion of a stretched allpass peaks at sample_rate / (2 *
    // stretch): about 3.4 kHz, the top of a spring tank's useful band.
    stretch_ = static_cast<int32_t>(sample_rate / 7000.0f + 0.5f);
    CONSTRAIN(stretch_, 1, kReverbMaxSpringStretch);

    // Lines 0, 1, 4 and 5 live in the small pool, the others share the large
    // one with the pre-delay and the diffusers.
    const int line_pool[kReverbNumLines] = { 1, 1, 0, 0, 1, 1, 0, 0 };
    const float max_modulation = std::max(std::max(
        kReverbRoom.modulation_ms, kReverbHall.modulation_ms),
        kReverbSpring.modulation_ms) * samples_per_ms_;
    used_[0] = used_[1] = 0;
    for (size_t i = 0; i < kReverbNumLines; ++i) {
      const float longest = std::max(std::max(
          kReverbRoom.line_ms[i], kReverbHall.line_ms[i]),
          kReverbSpring.line_ms[i]);
      Allocate(
          line_pool[i],
          longest * kReverbMaxSizeScale * samples_per_ms_ + max_modulation,
          &line_[i]);
    }
    Allocate(0, kReverbMaxEarlyMs * samples_per_ms_, &pre_delay_line_);
    const float diffuser_ms[kReverbNumDiffusers] = {
      4.77f, 3.59f, 2.73f, 1.83f
    };
    for (size_t i = 0; i < kReverbNumDiffusers; ++i) {
      diffuser_delay_[i] = static_cast<int32_t>(
          diffuser_ms[i] * samples_per_ms_);
      Allocate(0, float(diffuser_delay_[i]), &diffuser_line_[i]);
    }

    input_hp_.Init();
    input_lp_.Init();
    output_lp_[0].Init();
    output_lp_[1].Init();

    for (size_t i = 0; i < kReverbNumLines; ++i) {
      lfo_phase_[i] = float(i) * 0.125f;
    }
    for (size_t i = 0; i < 3; ++i) {
      angle_lfo_phase_[i] = float(i) * 0.33f;
    }
    Clear();
  }

  void Clear() {
    std::fill(&pool_[0][0], &pool_[0][kReverbLargePoolSize], 0);
    std::fill(&small_pool_[0], &small_pool_[kReverbSmallPoolSize], 0);
    std::fill(
        &chain_[0][0][0],
        &chain_[0][0][0] + sizeof(chain_) / sizeof(float),
        0.0f);
    std::fill(&absorption_state_[0], &absorption_state_[kReverbNumLines], 0.0f);
    input_hp_.Reset();
    input_lp_.Reset();
    output_lp_[0].Reset();
    output_lp_[1].Reset();
    write_ptr_ = 0;
    chain_index_ = 0;
    // The first block jumps to the parameters instead of gliding.
    snap_ = true;
  }

  // Dry/wet balance, same law as the original Beads reverb.
  inline void set_amount(float amount) {
    mix_ = amount;
  }

  inline void set_parameters(const ReverbParameters& parameters) {
    target_ = parameters;
  }

  inline uint32_t memory_used(int pool) const {
    return used_[pool];
  }

  void Process(FloatFrame* in_out, size_t size) {
    while (size) {
      size_t n = std::min(size, kReverbMaxBlockSize);
      ProcessBlock(in_out, n);
      in_out += n;
      size -= n;
    }
  }

 private:
  struct DelayLine {
    uint16_t* buffer;
    uint32_t mask;
    uint32_t base;
    int32_t length;
  };

  void Allocate(int pool, float length, DelayLine* line) {
    int32_t l = static_cast<int32_t>(length) + 4;
    line->buffer = pool_[pool];
    line->mask = (pool == 0 ? kReverbLargePoolSize : kReverbSmallPoolSize) - 1;
    line->base = used_[pool] + l + 1;
    line->length = l;
    used_[pool] += l + 2;
  }

  static inline float Decompress(uint16_t value) {
    return static_cast<float>(static_cast<int16_t>(value)) / 32768.0f;
  }

  // Rounds to the nearest step. Truncation would act as extra damping on
  // quiet tails, and cut them short.
  static inline uint16_t Compress(float value) {
    value *= 32768.0f;
    value += value >= 0.0f ? 0.5f : -0.5f;
    return static_cast<uint16_t>(
        stmlib::Clip16(static_cast<int32_t>(value)));
  }

  // The write pointer only goes up: the sample written at time t sits at
  // t + base, and a delay d reads t + base - d.
  static inline void Write(const DelayLine& line, uint32_t t, float value) {
    line.buffer[(t + line.base) & line.mask] = Compress(value);
  }

  static inline float Read(const DelayLine& line, uint32_t t, int32_t delay) {
    return Decompress(line.buffer[(t + line.base - delay) & line.mask]);
  }

  static inline float Read(const DelayLine& line, uint32_t t, float delay) {
    MAKE_INTEGRAL_FRACTIONAL(delay);
    const uint32_t i = t + line.base - delay_integral;
    const float a = Decompress(line.buffer[i & line.mask]);
    const float b = Decompress(line.buffer[(i - 1) & line.mask]);
    return a + (b - a) * delay_fractional;
  }

  // 2^x, about 1e-4 relative error.
  static inline float Exp2(float x) {
    if (x < -126.0f) {
      return 0.0f;
    }
    int32_t i = static_cast<int32_t>(x);
    if (x < static_cast<float>(i)) {
      --i;
    }
    const float f = x - static_cast<float>(i);
    union {
      float f;
      int32_t i;
    } r;
    r.f = 1.0f + f * (0.6960656f + f * (0.2244943f + f * 0.0794402f));
    r.i += i << 23;
    return r.f;
  }

  // Smooth LFO shape for a phase in [0, 1). Close to a cosine.
  static inline float Lfo(float phase) {
    float x = phase < 0.5f ? 4.0f * phase - 1.0f : 3.0f - 4.0f * phase;
    return x * (1.5f - 0.5f * x * x);
  }

  inline void Rotate(float* a, float* b, int level) const {
    const float c = rotation_cos_[level];
    const float s = rotation_sin_[level];
    const float x = *a;
    const float y = *b;
    *a = c * x - s * y;
    *b = s * x + c * y;
  }

  // Maps the knob to [0, 2] (room, hall, spring) with flat spots at the
  // three pure characters, so that they are easy to find by hand.
  static void Blend(float character, ReverbCharacter* c) {
    const float kFlat = 0.06f;
    const ReverbCharacter* a = &kReverbRoom;
    const ReverbCharacter* b = &kReverbHall;
    float t = (character - kFlat) / (0.5f - 2.0f * kFlat);
    if (character >= 0.5f) {
      a = &kReverbHall;
      b = &kReverbSpring;
      t = (character - 0.5f - kFlat) / (0.5f - 2.0f * kFlat);
    }
    CONSTRAIN(t, 0.0f, 1.0f);

#define BLEND(x) c->x = a->x + (b->x - a->x) * t
    for (size_t i = 0; i < kReverbNumLines; ++i) {
      BLEND(line_ms[i]);
    }
    BLEND(pre_delay_ms);
    BLEND(early_level);
    BLEND(early_span_ms);
    BLEND(decay_scale);
    for (size_t i = 0; i < 3; ++i) {
      BLEND(coupling[i]);
    }
    BLEND(second_group);
    BLEND(dispersion);
    BLEND(modulation_ms);
    BLEND(modulation_hz);
    BLEND(input_hp_hz);
    BLEND(input_lp_hz);
    BLEND(diffuser);
    BLEND(output_gain);
#undef BLEND
  }

  void UpdateParameters(size_t size) {
    const float block_time = float(size) / sample_rate_;

    // Size and character glide slowly (about 80 ms): turning them bends the
    // pitch of the tail, like a tape speed change.
    const float slow = snap_ ? 1.0f : std::min(block_time * 12.0f, 1.0f);
    const float fast = snap_ ? 1.0f : std::min(block_time * 50.0f, 1.0f);
    ONE_POLE(parameters_.character, target_.character, slow);
    ONE_POLE(parameters_.size, target_.size, slow);
    ONE_POLE(parameters_.decay, target_.decay, fast);
    ONE_POLE(parameters_.tone, target_.tone, fast);
    ONE_POLE(parameters_.diffusion, target_.diffusion, fast);

    ReverbCharacter c;
    Blend(parameters_.character, &c);

    // Size scales all lengths from 0.25x to 1.6x. Pre-delay and early
    // reflections follow more gently.
    const float log2_scale = -2.0f + 2.678f * parameters_.size;
    const float scale = Exp2(log2_scale);
    const float soft_scale = Exp2(0.7f * log2_scale);

    // Decay: 0.3 s to 36 s, times the character's own scale. Tone sets how
    // much faster the highs die (Nyquist over DC decay time, 0.06 to 0.9).
    const float t60 = 0.3f * Exp2(6.907f * parameters_.decay) * c.decay_scale;
    const float tone_ratio = 0.06f * Exp2(3.907f * parameters_.tone);
    const float diffusion = parameters_.diffusion * (2.0f - parameters_.diffusion);

    // Matrix angles, slowly modulated to break up resonances.
    const float angle_modulation = 0.05f * (0.25f + 0.75f * diffusion);
    const float angle_lfo_hz[3] = { 0.071f, 0.113f, 0.053f };
    for (size_t level = 0; level < 3; ++level) {
      angle_lfo_phase_[level] += angle_lfo_hz[level] * block_time;
      if (angle_lfo_phase_[level] >= 1.0f) {
        angle_lfo_phase_[level] -= 1.0f;
      }
      float theta = 0.7853982f * diffusion * c.coupling[level] + \
          angle_modulation * Lfo(angle_lfo_phase_[level]);
      float theta2 = theta * theta;
      float s = theta * (1.0f - theta2 * (0.16666667f - theta2 * 0.00833333f));
      float co = 1.0f - theta2 * (0.5f - theta2 * (
          0.04166667f - theta2 * 0.00138889f));
      // Keep the matrix exactly lossless despite the approximations.
      float norm = 1.0f / stmlib::Sqrt(s * s + co * co);
      rotation_sin_[level] = s * norm;
      rotation_cos_[level] = co * norm;
    }

    // Lines: delay with modulation, and the absorption filter (one pole,
    // with the decay times at DC and at Nyquist as end points).
    const float lfo_ratio[kReverbNumLines] = {
      1.0f, 1.37f, 0.71f, 1.93f, 0.83f, 1.19f, 1.61f, 0.59f
    };
    const float spring_spread[kReverbNumSpringLines] = {
      1.0f, 0.93f, 1.05f, 0.97f
    };
    const float modulation = c.modulation_ms * samples_per_ms_;
    const float decay_per_sample = -9.9658f / (t60 * sample_rate_);
    const float increment_scale = 1.0f / float(size);
    for (size_t i = 0; i < kReverbNumLines; ++i) {
      lfo_phase_[i] += c.modulation_hz * lfo_ratio[i] * block_time;
      if (lfo_phase_[i] >= 1.0f) {
        lfo_phase_[i] -= 1.0f;
      }

      const float loop = c.line_ms[i] * scale * samples_per_ms_;
      float delay = loop;
      if (i < kReverbNumSpringLines) {
        // The allpass cascade adds its own delay: take its low-frequency
        // group delay out of the line.
        const float a = c.dispersion * spring_spread[i];
        dispersion_[i] = a;
        delay -= float(kReverbNumSpringStages * stretch_) * \
            (1.0f - a) / (1.0f + a);
      }
      delay += modulation * Lfo(lfo_phase_[i]);
      CONSTRAIN(delay, kReverbMinDelay, float(line_[i].length - 1));
      if (snap_) {
        line_delay_[i] = delay;
      }
      line_delay_increment_[i] = (delay - line_delay_[i]) * increment_scale;

      const float g_dc = Exp2(decay_per_sample * loop);
      const float g_nyquist = Exp2(decay_per_sample * loop / tone_ratio);
      const float pole = (g_dc - g_nyquist) / (g_dc + g_nyquist);
      absorption_pole_[i] = pole;
      absorption_gain_[i] = g_dc * (1.0f - pole);
    }

    // Injection and output weights. Towards the spring end, lines 4 to 7
    // fade out.
    const float input_sign[kReverbNumLines] = {
      1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f
    };
    second_group_ = c.second_group;
    for (size_t i = 0; i < kReverbNumLines; ++i) {
      input_gain_[i] = input_sign[i] * (i < 4 ? 1.0f : second_group_);
    }
    output_norm_ = 1.0f / stmlib::Sqrt(
        2.0f + 2.0f * second_group_ * second_group_);

    // Pre-delay and early reflections.
    float pre_delay = c.pre_delay_ms * soft_scale * samples_per_ms_;
    CONSTRAIN(pre_delay, 1.0f, float(pre_delay_line_.length - 1));
    if (snap_) {
      pre_delay_ = pre_delay;
    }
    pre_delay_increment_ = (pre_delay - pre_delay_) * increment_scale;

    const float early_time[2][kReverbNumEarlyTaps] = {
      { 0.07f, 0.21f, 0.36f, 0.52f, 0.71f, 0.90f },
      { 0.12f, 0.27f, 0.43f, 0.60f, 0.77f, 0.95f }
    };
    const float span = c.early_span_ms * soft_scale;
    for (size_t channel = 0; channel < 2; ++channel) {
      for (size_t j = 0; j < kReverbNumEarlyTaps; ++j) {
        float d = (1.0f + early_time[channel][j] * span) * samples_per_ms_;
        CONSTRAIN(d, 1.0f, float(pre_delay_line_.length - 1));
        if (snap_) {
          early_delay_[channel][j] = d;
        }
        early_increment_[channel][j] = \
            (d - early_delay_[channel][j]) * increment_scale;
      }
    }
    early_level_ = c.early_level;
    early_feed_ = 0.35f * c.early_level;
    diffuser_gain_ = 0.7f * diffusion * c.diffuser;

    // Input band limits (the spring is thin and dark) and output tone.
    input_hp_.set_f<stmlib::FREQUENCY_DIRTY>(c.input_hp_hz / sample_rate_);
    input_lp_.set_f_q<stmlib::FREQUENCY_FAST>(
        std::min(c.input_lp_hz / sample_rate_, 0.35f), 0.7f);
    const float output_lp = std::min(
        2000.0f * Exp2(3.3219f * parameters_.tone) / sample_rate_, 0.35f);
    output_lp_[0].set_f<stmlib::FREQUENCY_FAST>(output_lp);
    output_lp_[1].set_f<stmlib::FREQUENCY_FAST>(output_lp);

    // Longer tails build up more energy: compensate part of it, so that the
    // REVERB knob keeps the same meaning at all decay settings.
    float loudness = Exp2(-0.35f * (6.907f * parameters_.decay - 3.06f));
    CONSTRAIN(loudness, 0.4f, 2.5f);

    const float kInputGain = 0.25f;
    const float kOutputGain = 1.8f;
    const float send_amount_sqrt = mix_ * (2.0f - mix_);
    send_target_ = send_amount_sqrt * (2.0f - send_amount_sqrt) * kInputGain;
    dry_target_ = (1.0f - mix_) * (1.0f + mix_);
    wet_target_ = kOutputGain * c.output_gain * loudness;

    snap_ = false;
  }

  void ProcessBlock(FloatFrame* in_out, size_t size) {
    UpdateParameters(size);

    stmlib::ParameterInterpolator send_modulation(&send_, send_target_, size);
    stmlib::ParameterInterpolator dry_modulation(&dry_, dry_target_, size);
    stmlib::ParameterInterpolator wet_modulation(&wet_, wet_target_, size);

    float tank_in[kReverbMaxBlockSize];
    float early_l[kReverbMaxBlockSize];
    float early_r[kReverbMaxBlockSize];

    const float early_gain[2][kReverbNumEarlyTaps] = {
      { 0.84f, -0.66f, 0.55f, -0.45f, 0.36f, -0.28f },
      { 0.80f, -0.62f, 0.58f, -0.42f, 0.33f, -0.26f }
    };

    // Input filters, pre-delay, early reflections and diffusers.
    uint32_t t = write_ptr_;
    for (size_t k = 0; k < size; ++k) {
      ++t;
      float x = (in_out[k].l + in_out[k].r) * send_modulation.Next();
      x = input_hp_.Process<stmlib::FILTER_MODE_HIGH_PASS>(x);
      x = input_lp_.Process<stmlib::FILTER_MODE_LOW_PASS>(x);
      Write(pre_delay_line_, t, x);

      float l = 0.0f;
      float r = 0.0f;
      for (size_t j = 0; j < kReverbNumEarlyTaps; ++j) {
        early_delay_[0][j] += early_increment_[0][j];
        early_delay_[1][j] += early_increment_[1][j];
        l += early_gain[0][j] * Read(pre_delay_line_, t, early_delay_[0][j]);
        r += early_gain[1][j] * Read(pre_delay_line_, t, early_delay_[1][j]);
      }
      early_l[k] = l;
      early_r[k] = r;

      pre_delay_ += pre_delay_increment_;
      x = Read(pre_delay_line_, t, pre_delay_) + early_feed_ * (l + r);
      for (size_t j = 0; j < kReverbNumDiffusers; ++j) {
        const float delayed = Read(diffuser_line_[j], t, diffuser_delay_[j]);
        const float v = x + diffuser_gain_ * delayed;
        Write(diffuser_line_[j], t, v);
        x = delayed - diffuser_gain_ * v;
      }
      tank_in[k] = x;
    }

    // Read the tank lines. All delays are longer than a block, so the whole
    // block can be read before anything is written.
    float y[kReverbNumLines][kReverbMaxBlockSize];
    for (size_t i = 0; i < kReverbNumLines; ++i) {
      const DelayLine& line = line_[i];
      const float increment = line_delay_increment_[i];
      float delay = line_delay_[i];
      uint32_t ti = write_ptr_;
      for (size_t k = 0; k < size; ++k) {
        ++ti;
        delay += increment;
        y[i][k] = Read(line, ti, delay);
      }
      line_delay_[i] = delay;
    }

    // Spring dispersion: stretched allpass cascades on lines 0 to 3. The
    // four cascades run side by side, so that the FPU pipeline is not
    // waiting on one long chain of dependent operations.
    int32_t index = chain_index_;
    const float a0 = dispersion_[0];
    const float a1 = dispersion_[1];
    const float a2 = dispersion_[2];
    const float a3 = dispersion_[3];
    for (size_t k = 0; k < size; ++k) {
      float s0 = y[0][k];
      float s1 = y[1][k];
      float s2 = y[2][k];
      float s3 = y[3][k];
      for (size_t stage = 0; stage < kReverbNumSpringStages; ++stage) {
        float* z = chain_[stage][index];
        const float v0 = s0 - a0 * z[0];
        const float v1 = s1 - a1 * z[1];
        const float v2 = s2 - a2 * z[2];
        const float v3 = s3 - a3 * z[3];
        s0 = a0 * v0 + z[0];
        s1 = a1 * v1 + z[1];
        s2 = a2 * v2 + z[2];
        s3 = a3 * v3 + z[3];
        z[0] = v0;
        z[1] = v1;
        z[2] = v2;
        z[3] = v3;
      }
      y[0][k] = s0;
      y[1][k] = s1;
      y[2][k] = s2;
      y[3][k] = s3;
      if (++index >= stretch_) {
        index = 0;
      }
    }
    chain_index_ = index;

    // Absorption, output taps, mixing matrix, injection.
    float state[kReverbNumLines];
    std::copy(
        &absorption_state_[0],
        &absorption_state_[kReverbNumLines],
        &state[0]);
    t = write_ptr_;
    for (size_t k = 0; k < size; ++k) {
      ++t;
      float s[kReverbNumLines];
      for (size_t i = 0; i < kReverbNumLines; ++i) {
        state[i] = absorption_gain_[i] * y[i][k] + \
            absorption_pole_[i] * state[i];
        s[i] = state[i];
      }

      float l = (s[0] + s[2] + second_group_ * (s[4] + s[6])) * output_norm_;
      float r = (s[1] + s[3] + second_group_ * (s[5] + s[7])) * output_norm_;

      Rotate(&s[0], &s[1], 0);
      Rotate(&s[2], &s[3], 0);
      Rotate(&s[4], &s[5], 0);
      Rotate(&s[6], &s[7], 0);
      Rotate(&s[0], &s[2], 1);
      Rotate(&s[1], &s[3], 1);
      Rotate(&s[4], &s[6], 1);
      Rotate(&s[5], &s[7], 1);
      Rotate(&s[0], &s[4], 2);
      Rotate(&s[1], &s[5], 2);
      Rotate(&s[2], &s[6], 2);
      Rotate(&s[3], &s[7], 2);

      const float x = tank_in[k];
      for (size_t i = 0; i < kReverbNumLines; ++i) {
        Write(line_[i], t, s[i] + input_gain_[i] * x);
      }

      l += early_l[k] * early_level_;
      r += early_r[k] * early_level_;
      l = output_lp_[0].Process<stmlib::FILTER_MODE_LOW_PASS>(l);
      r = output_lp_[1].Process<stmlib::FILTER_MODE_LOW_PASS>(r);

      const float dry = dry_modulation.Next();
      const float wet = wet_modulation.Next();
      in_out[k].l = in_out[k].l * dry + l * wet;
      in_out[k].r = in_out[k].r * dry + r * wet;
    }
    std::copy(&state[0], &state[kReverbNumLines], &absorption_state_[0]);
    write_ptr_ = t;
  }

  uint16_t* pool_[2];
  uint32_t used_[2];
  uint16_t small_pool_[kReverbSmallPoolSize];

  DelayLine line_[kReverbNumLines];
  DelayLine pre_delay_line_;
  DelayLine diffuser_line_[kReverbNumDiffusers];
  int32_t diffuser_delay_[kReverbNumDiffusers];

  uint32_t write_ptr_;
  float sample_rate_;
  float samples_per_ms_;
  bool snap_;

  ReverbParameters target_;
  ReverbParameters parameters_;

  float mix_;
  float send_;
  float dry_;
  float wet_;
  float send_target_;
  float dry_target_;
  float wet_target_;

  float line_delay_[kReverbNumLines];
  float line_delay_increment_[kReverbNumLines];
  float lfo_phase_[kReverbNumLines];
  float absorption_gain_[kReverbNumLines];
  float absorption_pole_[kReverbNumLines];
  float absorption_state_[kReverbNumLines];
  float input_gain_[kReverbNumLines];
  float second_group_;
  float output_norm_;

  float rotation_cos_[3];
  float rotation_sin_[3];
  float angle_lfo_phase_[3];

  int32_t stretch_;
  int32_t chain_index_;
  float dispersion_[kReverbNumSpringLines];
  float chain_[kReverbNumSpringStages][kReverbMaxSpringStretch][
      kReverbNumSpringLines];

  float pre_delay_;
  float pre_delay_increment_;
  float early_delay_[2][kReverbNumEarlyTaps];
  float early_increment_[2][kReverbNumEarlyTaps];
  float early_level_;
  float early_feed_;
  float diffuser_gain_;

  stmlib::OnePole input_hp_;
  stmlib::Svf input_lp_;
  stmlib::OnePole output_lp_[2];

  DISALLOW_COPY_AND_ASSIGN(MorphReverb);
};

}  // namespace beads

#endif  // BEADS_DSP_FX_MORPH_REVERB_H_
