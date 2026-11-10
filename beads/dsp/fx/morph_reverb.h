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
// The input is stereo: the left channel goes through its own pre-delay and
// diffusers into the even lines, the right channel into the odd ones. With
// the same signal on both sides (a single input patched on the module), it
// behaves exactly like a mono input.
//
// Early reflections are a sparse set of taps on the pre-delay lines, which
// also feed the network through four allpass diffusers per side.
//
// The diffusion knob has two halves. On the left half the matrix angles and
// the input diffusers go from separate echoes to a smooth tail. On the right
// half the matrix stays fully mixed, three long allpass diffusers per side
// fade in to blur attacks into a swell, and the early reflections fade out.
//
// Ducking lowers the wet signal while the input is loud, and lets the tail
// come back when it gets quiet.
//
// When the module runs at 48 kHz, the reverb runs at 24 kHz, between halfband
// filters, for half the cost. The absorption filters and the interpolation of
// the tank lines are set so that the decay keeps the same color up to 8 kHz.

#ifndef BEADS_DSP_FX_MORPH_REVERB_H_
#define BEADS_DSP_FX_MORPH_REVERB_H_

#include "stmlib/stmlib.h"

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"

#include "beads/dsp/frame.h"
#include "beads/dsp/itcm.h"

namespace beads {

// The reverb runs once per audio block, and the instruction cache is much
// smaller than the audio code: unrolled loops would cost more in cache misses
// than they save. GCC only; the host build ignores it.
#if defined(__GNUC__) && !defined(__clang__)
#define REVERB_COMPACT __attribute__ ((optimize ("no-unroll-loops")))
#else
#define REVERB_COMPACT
#endif  // __GNUC__ && !__clang__

// Controls of the shift layer, all in [0, 1].
struct ReverbParameters {
  float character = 0.5f;  // 0: room, 0.5: hall, 1: spring.
  float size = 0.5f;
  float decay = 0.5f;
  float tone = 0.55f;
  float diffusion = 0.5f;  // 0.5: smooth, the most diffusion before the wash.
  float ducking = 0.0f;
};

const size_t kReverbNumLines = 8;
const size_t kReverbNumSpringLines = 4;
const size_t kReverbNumSpringStages = 24;
const int32_t kReverbMaxSpringStretch = 7;
const size_t kReverbNumEarlyTaps = 6;
const size_t kReverbNumDiffusers = 4;
const size_t kReverbNumWashDiffusers = 3;
const size_t kReverbMaxBlockSize = 24;

// Above this rate, the reverb runs at half the rate between a halfband
// decimator and interpolator. That halves its cost, so that the grains keep
// the CPU time they had with the original Beads reverb. The wet signal keeps
// everything up to about 10 kHz; at the default tone it stops around 7 kHz
// anyway.
const float kReverbHalfRateThreshold = 40000.0f;

// Halfband FIR, 31 taps (Kaiser window, beta 6.5). At 48 kHz: flat to 9 kHz,
// -0.5 dB at 10 kHz, 48 dB down from 15 kHz. The center tap is 0.5, and every
// other tap is zero: these are the taps 1, 3, 5... away from the center.
const size_t kReverbHalfbandPairs = 8;
const float kReverbHalfband[kReverbHalfbandPairs] = {
  0.314066625f, -0.094033296f, 0.045267672f, -0.022881134f,
  0.010840469f, -0.004432013f, 0.001371296f, -0.000199618f
};
const size_t kReverbDecimatorHistory = 4 * kReverbHalfbandPairs - 3;
const size_t kReverbInterpolatorHistory = 2 * kReverbHalfbandPairs - 1;

// With the REVERB knob down and the tail gone, the reverb is skipped
// altogether. Gone is below -60 dB for half a second: rounding in the 16-bit
// memory leaves the tank humming around -70 dB, it never reaches zero.
const float kReverbOffAmount = 0.001f;
const float kReverbSilence = 1.0e-3f;
const float kReverbQuietTime = 0.5f;

// The large pool is the buffer passed to Init (SRAM4 on the module). The
// two small pools are member arrays, which end up in DTCM with the processor.
const size_t kReverbNumPools = 3;
constexpr uint32_t kReverbPoolSize[kReverbNumPools] = { 32768, 16384, 8192 };

// Delay lines are read in blocks, so they must never be shorter than a block.
const float kReverbMinDelay = float(kReverbMaxBlockSize + 2);

// Levels in the 16-bit memory, made up for at the output: of the send (pre-
// delay, diffusers), and again of the tank lines. Dense grains into a long
// decay otherwise go past full scale and clip.
const float kReverbSendHeadroom = 0.7f;
const float kReverbTankHeadroom = 0.7f;
const float kReverbMaxSizeScale = 1.6f;
const float kReverbMaxEarlyMs = 100.0f;


// Ducking: envelope follower rates (per second) and the input peak level at
// which the full gain reduction is reached.
#ifndef DUCK_ATTACK
#define DUCK_ATTACK 200.0f
#endif
#ifndef DUCK_RELEASE
#define DUCK_RELEASE 3.0f
#endif
#ifndef DUCK_SENSITIVITY
#define DUCK_SENSITIVITY 3.0f
#endif

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
  float wash;  // Amount of the long diffusers at the end of the knob.
  float output_gain;
};

const ReverbCharacter kReverbRoom = {
  { 8.0f, 11.5f, 15.5f, 22.0f, 9.5f, 13.5f, 18.5f, 25.5f },
  2.0f, 0.6f, 35.0f, 0.35f,
  { 1.0f, 1.0f, 1.0f }, 1.0f, 0.0f,
  0.08f, 0.35f, 90.0f, 16000.0f, 0.9f, 1.0f, 0.8f
};

const ReverbCharacter kReverbHall = {
  { 31.0f, 47.0f, 61.0f, 83.0f, 37.0f, 53.0f, 73.0f, 97.0f },
  28.0f, 0.3f, 70.0f, 1.0f,
  { 1.0f, 1.0f, 1.0f }, 1.0f, 0.0f,
  0.45f, 0.5f, 25.0f, 14000.0f, 1.0f, 1.0f, 1.0f
};

// Lines 4 to 7 keep their hall lengths: they are silent at this end.
const ReverbCharacter kReverbSpring = {
  { 55.0f, 63.0f, 71.0f, 79.0f, 37.0f, 53.0f, 73.0f, 97.0f },
  18.0f, 0.0f, 40.0f, 0.9f,
  { 0.7f, 0.7f, 0.0f }, 0.0f, 0.7f,
  0.15f, 2.5f, 160.0f, 3200.0f, 0.25f, 0.7f, 1.45f
};

class MorphReverb {
 public:
  MorphReverb() { }
  ~MorphReverb() { }

  void Init(uint16_t* buffer, float sample_rate) {
    pool_[0] = buffer;
    pool_[1] = small_pool_;
    pool_[2] = smaller_pool_;
    mix_ = 0.0f;
    dry_ = 0.0f;
    wet_ = 0.0f;
    send_ = 0.0f;
    duck_envelope_ = 0.0f;
    Reset(sample_rate);
  }

  // Also clears the tank: delay lengths depend on the sample rate.
  void Reset(float sample_rate) {
    factor_ = sample_rate > kReverbHalfRateThreshold ? 2 : 1;
    quiet_threshold_ = static_cast<int32_t>(sample_rate * kReverbQuietTime);
    sample_rate /= float(factor_);
    sample_rate_ = sample_rate;
    samples_per_ms_ = sample_rate * 0.001f;
    // The dispersion of a stretched allpass peaks at sample_rate / (2 *
    // stretch): 3 to 3.4 kHz, the top of a spring tank's useful band. The
    // number of stages keeps the cascade about 3.5 ms long, as 24 stages of
    // 7 samples at 48 kHz.
    stretch_ = static_cast<int32_t>(sample_rate / 6500.0f + 0.5f);
    CONSTRAIN(stretch_, 1, kReverbMaxSpringStretch);
    num_stages_ = static_cast<int32_t>(
        0.0035f * sample_rate / float(stretch_) + 0.5f);
    CONSTRAIN(num_stages_, 1, int32_t(kReverbNumSpringStages));

    // Lines 0, 1, 4 and 5 live in the small pool, the others share the large
    // one with the pre-delay and the diffusers. At half rate all the lines
    // fit in the two small pools, which are faster to reach.
    const int line_pool[2][kReverbNumLines] = {
      { 1, 1, 0, 0, 1, 1, 0, 0 },
      { 1, 1, 1, 1, 1, 1, 2, 2 }
    };
    const float max_modulation = std::max(std::max(
        kReverbRoom.modulation_ms, kReverbHall.modulation_ms),
        kReverbSpring.modulation_ms) * samples_per_ms_;
    std::fill(&used_[0], &used_[kReverbNumPools], 0);
    for (size_t i = 0; i < kReverbNumLines; ++i) {
      const float longest = std::max(std::max(
          kReverbRoom.line_ms[i], kReverbHall.line_ms[i]),
          kReverbSpring.line_ms[i]);
      Allocate(
          line_pool[factor_ - 1][i],
          longest * kReverbMaxSizeScale * samples_per_ms_ + max_modulation,
          &line_[i]);
    }
    // At full rate, the left side keeps the pools of the mono version and
    // the right side lives in the third pool. At half rate, both sides share
    // the large pool.
    const float diffuser_ms[kReverbNumDiffusers] = {
      4.77f, 3.59f, 2.73f, 1.83f
    };
    const float wash_ms[2][kReverbNumWashDiffusers] = {
      { 8.9f, 13.3f, 19.7f },
      { 9.7f, 12.1f, 21.1f }
    };
    for (size_t side = 0; side < 2; ++side) {
      const int pool = side == 0 || factor_ == 2 ? 0 : 2;
      Allocate(pool, kReverbMaxEarlyMs * samples_per_ms_,
               &pre_delay_line_[side]);
      for (size_t i = 0; i < kReverbNumDiffusers; ++i) {
        diffuser_delay_[i] = static_cast<int32_t>(
            diffuser_ms[i] * samples_per_ms_);
        Allocate(pool, float(diffuser_delay_[i]), &diffuser_line_[side][i]);
      }
      for (size_t i = 0; i < kReverbNumWashDiffusers; ++i) {
        wash_delay_[side][i] = static_cast<int32_t>(
            wash_ms[side][i] * samples_per_ms_);
        Allocate(pool, float(wash_delay_[side][i]), &wash_line_[side][i]);
      }
      input_hp_[side].Init();
      input_lp_[side].Init();
      output_lp_[side].Init();
    }

    for (size_t i = 0; i < kReverbNumLines; ++i) {
      lfo_phase_[i] = float(i) * 0.125f;
    }
    for (size_t i = 0; i < 3; ++i) {
      angle_lfo_phase_[i] = float(i) * 0.33f;
    }
    Clear();
  }

  void Clear() {
    for (size_t i = 0; i < kReverbNumPools; ++i) {
      std::fill(&pool_[i][0], &pool_[i][kReverbPoolSize[i]], 0);
    }
    std::fill(
        &chain_[0][0][0],
        &chain_[0][0][0] + sizeof(chain_) / sizeof(float),
        0.0f);
    std::fill(&chain_offset_[0], &chain_offset_[kReverbMaxSpringStretch], 0);
    chain_rotated_ = false;
    std::fill(&absorption_state_[0], &absorption_state_[kReverbNumLines], 0.0f);
    for (size_t side = 0; side < 2; ++side) {
      input_hp_[side].Reset();
      input_lp_[side].Reset();
      output_lp_[side].Reset();
    }
    std::fill(
        &decimator_history_[0][0],
        &decimator_history_[0][0] + 2 * kReverbDecimatorHistory,
        0.0f);
    std::fill(
        &interpolator_history_[0][0],
        &interpolator_history_[0][0] + 2 * kReverbInterpolatorHistory,
        0.0f);
    quiet_samples_ = 0;
    duck_envelope_ = 0.0f;
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

  // Shape of the left half of the diffusion knob: from the knob position in
  // [0, 1] to the amount of matrix mixing and input diffusion. The echo
  // density of the hall rises evenly over the whole half.
  static inline float DiffusionCurve(float u) {
    return u * (0.45f + 0.55f * u * u);
  }

  // At half rate, size must be even (Beads blocks are 12 samples). An odd
  // last sample only gets the dry level.
  REVERB_COMPACT void Process(FloatFrame* in_out, size_t size) {
    if (mix_ < kReverbOffAmount && quiet_samples_ >= quiet_threshold_) {
      return;
    }
    while (size) {
      size_t n = std::min(size, factor_ * kReverbMaxBlockSize);
      FloatFrame wet[2 * kReverbMaxBlockSize];
      if (factor_ == 2) {
        n &= ~size_t(1);
        if (n == 0) {
          in_out[0].l *= dry_;
          in_out[0].r *= dry_;
          return;
        }
        FloatFrame half_in[kReverbMaxBlockSize];
        FloatFrame half_wet[kReverbMaxBlockSize];
        Decimate(in_out, half_in, n);
        ProcessBlock(half_in, half_wet, n / 2);
        Interpolate(half_wet, wet, n / 2);
      } else {
        ProcessBlock(in_out, wet, n);
      }

      float peak = 0.0f;
      stmlib::ParameterInterpolator dry_modulation(&dry_, dry_target_, n);
      for (size_t k = 0; k < n; ++k) {
        const float dry = dry_modulation.Next();
        in_out[k].l = in_out[k].l * dry + wet[k].l;
        in_out[k].r = in_out[k].r * dry + wet[k].r;
        peak = std::max(peak, std::max(fabsf(wet[k].l), fabsf(wet[k].r)));
      }
      if (mix_ < kReverbOffAmount && peak < kReverbSilence) {
        quiet_samples_ += n;
      } else {
        quiet_samples_ = 0;
      }
      in_out += n;
      size -= n;
    }
  }

 private:
  // Polyphase halfband decimator, from size frames to size / 2.
  REVERB_COMPACT void Decimate(
      const FloatFrame* in, FloatFrame* out, size_t size) {
    float x[kReverbDecimatorHistory + 2 * kReverbMaxBlockSize];
    for (size_t channel = 0; channel < 2; ++channel) {
      float* history = decimator_history_[channel];
      std::copy(&history[0], &history[kReverbDecimatorHistory], &x[0]);
      for (size_t k = 0; k < size; ++k) {
        x[kReverbDecimatorHistory + k] = channel ? in[k].r : in[k].l;
      }
      for (size_t j = 0; j < size / 2; ++j) {
        const float* center = &x[2 * j + 2 * kReverbHalfbandPairs - 1];
        float s = 0.5f * center[0];
        for (size_t i = 0; i < kReverbHalfbandPairs; ++i) {
          s += kReverbHalfband[i] * (
              center[-1 - 2 * int32_t(i)] + center[1 + 2 * i]);
        }
        if (channel) {
          out[j].r = s;
        } else {
          out[j].l = s;
        }
      }
      std::copy(
          &x[size], &x[size + kReverbDecimatorHistory], &history[0]);
    }
  }

  // Polyphase halfband interpolator, from size frames to 2 * size. One
  // phase is a plain delay, the other one carries all the taps.
  REVERB_COMPACT void Interpolate(
      const FloatFrame* in, FloatFrame* out, size_t size) {
    float w[kReverbInterpolatorHistory + kReverbMaxBlockSize];
    for (size_t channel = 0; channel < 2; ++channel) {
      float* history = interpolator_history_[channel];
      std::copy(&history[0], &history[kReverbInterpolatorHistory], &w[0]);
      for (size_t k = 0; k < size; ++k) {
        w[kReverbInterpolatorHistory + k] = channel ? in[k].r : in[k].l;
      }
      for (size_t q = 0; q < size; ++q) {
        const float* center = &w[q + kReverbHalfbandPairs - 1];
        float s = 0.0f;
        for (size_t i = 0; i < kReverbHalfbandPairs; ++i) {
          s += kReverbHalfband[i] * (center[1 + i] + center[-int32_t(i)]);
        }
        if (channel) {
          out[2 * q].r = 2.0f * s;
          out[2 * q + 1].r = center[1];
        } else {
          out[2 * q].l = 2.0f * s;
          out[2 * q + 1].l = center[1];
        }
      }
      std::copy(
          &w[size], &w[size + kReverbInterpolatorHistory], &history[0]);
    }
  }

  struct DelayLine {
    uint16_t* buffer;
    uint32_t mask;
    uint32_t base;
    int32_t length;
  };

  void Allocate(int pool, float length, DelayLine* line) {
    int32_t l = static_cast<int32_t>(length) + 4;
    line->buffer = pool_[pool];
    line->mask = kReverbPoolSize[pool] - 1;
    line->base = used_[pool] + l + 1;
    line->length = l;
    used_[pool] += l + 2;
  }

  static inline float Decompress(uint16_t value) {
#ifdef __arm__
    // Conversion and scaling in one instruction: a fixed point number with
    // 15 fractional bits.
    float result;
    const int32_t x = static_cast<int16_t>(value);
    __asm__("vmov %0, %1\n\tvcvt.f32.s32 %0, %0, #15"
            : "=&t" (result) : "r" (x));
    return result;
#else
    return static_cast<float>(static_cast<int16_t>(value)) / 32768.0f;
#endif  // __arm__
  }

  // Rounds to the nearest step. Truncation would act as extra damping on
  // quiet tails, and cut them short.
  static inline uint16_t Compress(float value) {
    value *= 32768.0f;
#ifdef __arm__
    // vcvtr rounds with the FPSCR mode: to nearest, the reset default.
    float rounded;
    int32_t x;
    __asm__("vcvtr.s32.f32 %1, %2\n\tvmov %0, %1"
            : "=r" (x), "=&t" (rounded) : "t" (value));
#else
    value += value >= 0.0f ? 0.5f : -0.5f;
    const int32_t x = static_cast<int32_t>(value);
#endif  // __arm__
    return static_cast<uint16_t>(stmlib::Clip16(x));
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

  // For the tank lines, where every pass through the loop goes through the
  // interpolation again. Linear interpolation dulls the highs a little on
  // each pass, and more so at half rate: at 24 kHz, Hermite loses about as
  // much up to 6 kHz as linear did at 48 kHz. The sample after the tap is
  // read too, so the delay must be at least a block plus 2.
  static inline float ReadHermite(
      const DelayLine& line, uint32_t t, float delay) {
    MAKE_INTEGRAL_FRACTIONAL(delay);
    const uint32_t i = t + line.base - delay_integral;
    const float xm1 = Decompress(line.buffer[(i + 1) & line.mask]);
    const float x0 = Decompress(line.buffer[i & line.mask]);
    const float x1 = Decompress(line.buffer[(i - 1) & line.mask]);
    const float x2 = Decompress(line.buffer[(i - 2) & line.mask]);
    const float c = (x1 - xm1) * 0.5f;
    const float v = x0 - x1;
    const float w = c + v;
    const float a = w + v + (x2 - x0) * 0.5f;
    const float b_neg = w + a;
    const float f = delay_fractional;
    return (((a * f) - b_neg) * f + c) * f + x0;
  }

  // Back to stage k at row k, before the cascade runs again. About 2.7 kB
  // of copies, once, when the character enters the spring half.
  void UnrotateChain() {
    float column[kReverbNumSpringStages][kReverbNumSpringLines];
    for (int32_t slot = 0; slot < kReverbMaxSpringStretch; ++slot) {
      const int32_t offset = chain_offset_[slot];
      if (offset == 0) {
        continue;
      }
      for (size_t stage = 0; stage < size_t(num_stages_); ++stage) {
        const size_t row = (stage + offset) % num_stages_;
        std::copy(
            &chain_[row][slot][0],
            &chain_[row][slot][kReverbNumSpringLines],
            &column[stage][0]);
      }
      for (size_t stage = 0; stage < size_t(num_stages_); ++stage) {
        std::copy(
            &column[stage][0],
            &column[stage][kReverbNumSpringLines],
            &chain_[stage][slot][0]);
      }
      chain_offset_[slot] = 0;
    }
    chain_rotated_ = false;
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
    BLEND(wash);
    BLEND(output_gain);
#undef BLEND
  }

  ITCM_SECTION(reverb_parameters) REVERB_COMPACT void UpdateParameters(
      size_t size, float input_peak) {
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
    ONE_POLE(parameters_.ducking, target_.ducking, fast);

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
    // Left half of the knob: separate echoes to smooth. Right half: smooth
    // to a wash.
    const float diffusion = DiffusionCurve(
        std::min(2.0f * parameters_.diffusion, 1.0f));
    const float smear = std::max(2.0f * parameters_.diffusion - 1.0f, 0.0f);

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
        delay -= float(num_stages_ * stretch_) * \
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
      float pole = (g_dc - g_nyquist) / (g_dc + g_nyquist);
      if (factor_ == 2) {
        // The end points stay those of the full rate filter: DC and 24 kHz.
        // At half rate the pole gives the same gain at 6 kHz, half the new
        // Nyquist, and the two filters stay close over the whole band.
        const float r2 = (1.0f - pole) * (1.0f - pole) / (
            1.0f - 1.4142136f * pole + pole * pole);
        const float a = 1.0f - r2;
        pole = a / (1.0f + stmlib::Sqrt(1.0f - a * a));
      }
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
      input_gain_[i] = input_sign[i] * (i < 4 ? 1.0f : second_group_) * \
          kReverbTankHeadroom;
    }
    output_norm_ = 1.0f / (kReverbTankHeadroom * stmlib::Sqrt(
        2.0f + 2.0f * second_group_ * second_group_));

    // Pre-delay and early reflections.
    float pre_delay = c.pre_delay_ms * soft_scale * samples_per_ms_;
    CONSTRAIN(pre_delay, 1.0f, float(pre_delay_line_[0].length - 1));
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
        CONSTRAIN(d, 1.0f, float(pre_delay_line_[0].length - 1));
        if (snap_) {
          early_delay_[channel][j] = d;
        }
        early_increment_[channel][j] = \
            (d - early_delay_[channel][j]) * increment_scale;
      }
    }
    early_level_ = c.early_level * (1.0f - 0.85f * smear);
    early_feed_ = 0.35f * c.early_level;
    diffuser_gain_ = 0.7f * diffusion * c.diffuser;
    // Equal power crossfade: the diffused signal is almost uncorrelated with
    // the direct one.
    const float wash_mix = smear * c.wash;
    dry_wash_ = stmlib::Sqrt(1.0f - wash_mix);
    wet_wash_ = stmlib::Sqrt(wash_mix);
    wash_gain_ = 0.5f + 0.2f * smear;

    // Input band limits (the spring is thin and dark) and output tone. At
    // half rate the low-pass filters go up to 10 kHz, which takes a more
    // accurate tangent so close to Nyquist.
    input_hp_[0].set_f<stmlib::FREQUENCY_DIRTY>(c.input_hp_hz / sample_rate_);
    input_hp_[1].set_f<stmlib::FREQUENCY_DIRTY>(c.input_hp_hz / sample_rate_);
    const float tone_hz = 2000.0f * Exp2(3.3219f * parameters_.tone);
    if (factor_ == 2) {
      input_lp_[0].set_f_q<stmlib::FREQUENCY_ACCURATE>(
          std::min(c.input_lp_hz / sample_rate_, 0.42f), 0.7f);
      const float output_lp = std::min(tone_hz / sample_rate_, 0.42f);
      output_lp_[0].set_f<stmlib::FREQUENCY_ACCURATE>(output_lp);
      output_lp_[1].set_f<stmlib::FREQUENCY_ACCURATE>(output_lp);
    } else {
      input_lp_[0].set_f_q<stmlib::FREQUENCY_FAST>(
          std::min(c.input_lp_hz / sample_rate_, 0.35f), 0.7f);
      const float output_lp = std::min(tone_hz / sample_rate_, 0.35f);
      output_lp_[0].set_f<stmlib::FREQUENCY_FAST>(output_lp);
      output_lp_[1].set_f<stmlib::FREQUENCY_FAST>(output_lp);
    }
    input_lp_[1].set(input_lp_[0]);

    // Longer tails build up more energy: compensate part of it, so that the
    // REVERB knob keeps the same meaning at all decay settings. The cut for
    // long tails goes on the send, where it keeps the tank from clipping, and
    // the boost for short ones on the output.
    float loudness = Exp2(-0.35f * (6.907f * parameters_.decay - 3.06f));
    CONSTRAIN(loudness, 0.4f, 2.5f);
    const float send_loudness = std::min(loudness, 1.0f);
    const float wet_loudness = std::max(loudness, 1.0f);

    const float kInputGain = 0.25f;
    const float kOutputGain = 1.8f;
    const float send_amount_sqrt = mix_ * (2.0f - mix_);
    send_target_ = send_amount_sqrt * (2.0f - send_amount_sqrt) * \
        kInputGain * send_loudness * kReverbSendHeadroom;
    dry_target_ = (1.0f - mix_) * (1.0f + mix_);
    wet_target_ = kOutputGain * c.output_gain * wet_loudness / \
        kReverbSendHeadroom;

    // Ducking: a peak follower on the input, fast attack and slow release.
    // Its gain reduction reaches 24 dB at full depth with a loud input.
    const float duck_coefficient = input_peak > duck_envelope_
        ? DUCK_ATTACK * block_time
        : DUCK_RELEASE * block_time;
    duck_envelope_ += std::min(duck_coefficient, 1.0f) * (
        input_peak - duck_envelope_);
    const float duck = std::min(duck_envelope_ * DUCK_SENSITIVITY, 1.0f);
    wet_target_ *= Exp2(-4.0f * parameters_.ducking * duck);

    snap_ = false;
  }

  // Writes the wet signal only: the dry level is applied by Process, at the
  // full rate.
  ITCM_SECTION(reverb_block) REVERB_COMPACT void ProcessBlock(
      const FloatFrame* in, FloatFrame* wet_out, size_t size) {
    float input_peak = 0.0f;
    for (size_t k = 0; k < size; ++k) {
      input_peak = std::max(input_peak, std::max(
          fabsf(in[k].l), fabsf(in[k].r)));
    }
    UpdateParameters(size, input_peak);

    stmlib::ParameterInterpolator send_modulation(&send_, send_target_, size);
    stmlib::ParameterInterpolator wet_modulation(&wet_, wet_target_, size);

    float tank_in[2][kReverbMaxBlockSize];
    float early_l[kReverbMaxBlockSize];
    float early_r[kReverbMaxBlockSize];

    const float early_gain[2][kReverbNumEarlyTaps] = {
      { 0.84f, -0.66f, 0.55f, -0.45f, 0.36f, -0.28f },
      { 0.80f, -0.62f, 0.58f, -0.42f, 0.33f, -0.26f }
    };
    // Two reflections of each side come from the other side's input.
    const DelayLine* early_source[2][kReverbNumEarlyTaps] = {
      { &pre_delay_line_[0], &pre_delay_line_[0], &pre_delay_line_[1],
        &pre_delay_line_[0], &pre_delay_line_[0], &pre_delay_line_[1] },
      { &pre_delay_line_[1], &pre_delay_line_[1], &pre_delay_line_[0],
        &pre_delay_line_[1], &pre_delay_line_[1], &pre_delay_line_[0] }
    };

    // At the spring end, the early reflections are silent: skip them.
    const bool early = early_level_ != 0.0f || early_feed_ != 0.0f;
    if (!early) {
      for (size_t j = 0; j < kReverbNumEarlyTaps; ++j) {
        early_delay_[0][j] += early_increment_[0][j] * float(size);
        early_delay_[1][j] += early_increment_[1][j] * float(size);
      }
    }

    // Input filters, pre-delay, early reflections and diffusers. Twice the
    // channel is the same scale as the sum of both in the mono version.
    uint32_t t = write_ptr_;
    for (size_t k = 0; k < size; ++k) {
      ++t;
      const float send = 2.0f * send_modulation.Next();
      float x[2] = { in[k].l * send, in[k].r * send };
      for (size_t side = 0; side < 2; ++side) {
        x[side] = input_hp_[side].Process<stmlib::FILTER_MODE_HIGH_PASS>(
            x[side]);
        x[side] = input_lp_[side].Process<stmlib::FILTER_MODE_LOW_PASS>(
            x[side]);
        Write(pre_delay_line_[side], t, x[side]);
      }

      float l = 0.0f;
      float r = 0.0f;
      for (size_t j = 0; early && j < kReverbNumEarlyTaps; ++j) {
        early_delay_[0][j] += early_increment_[0][j];
        early_delay_[1][j] += early_increment_[1][j];
        l += early_gain[0][j] * Read(
            *early_source[0][j], t, early_delay_[0][j]);
        r += early_gain[1][j] * Read(
            *early_source[1][j], t, early_delay_[1][j]);
      }
      early_l[k] = l;
      early_r[k] = r;

      pre_delay_ += pre_delay_increment_;
      const float early_feed = early_feed_ * (l + r);
      for (size_t side = 0; side < 2; ++side) {
        float v = Read(pre_delay_line_[side], t, pre_delay_) + early_feed;
        for (size_t j = 0; j < kReverbNumDiffusers; ++j) {
          const DelayLine& line = diffuser_line_[side][j];
          const float delayed = Read(line, t, diffuser_delay_[j]);
          const float w = v + diffuser_gain_ * delayed;
          Write(line, t, w);
          v = delayed - diffuser_gain_ * w;
        }
        if (wet_wash_ == 0.0f) {
          // Left of noon, the long diffusers only fill up with silence, so
          // that they fade in later without a burst of stale samples.
          for (size_t j = 0; j < kReverbNumWashDiffusers; ++j) {
            Write(wash_line_[side][j], t, 0.0f);
          }
          tank_in[side][k] = v;
          continue;
        }
        float washed = v;
        for (size_t j = 0; j < kReverbNumWashDiffusers; ++j) {
          const DelayLine& line = wash_line_[side][j];
          const float delayed = Read(line, t, wash_delay_[side][j]);
          const float w = washed + wash_gain_ * delayed;
          Write(line, t, w);
          washed = delayed - wash_gain_ * w;
        }
        tank_in[side][k] = dry_wash_ * v + wet_wash_ * washed;
      }
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
        y[i][k] = ReadHermite(line, ti, delay);
      }
      line_delay_[i] = delay;
    }

    // Spring dispersion: stretched allpass cascades on lines 0 to 3. The
    // four cascades run side by side, so that the FPU pipeline is not
    // waiting on one long chain of dependent operations.
    int32_t index = chain_index_;
    if (dispersion_[0] == 0.0f) {
      // Room to hall: no dispersion, each stage is a plain delay, and one
      // step of the cascade shifts the column of the current slot down by
      // one stage. Rotating the column gives the same result for a fraction
      // of the work.
      for (size_t k = 0; k < size; ++k) {
        int32_t offset = chain_offset_[index] - 1;
        if (offset < 0) {
          offset += num_stages_;
        }
        chain_offset_[index] = offset;
        float* z = chain_[offset][index];
        for (size_t i = 0; i < kReverbNumSpringLines; ++i) {
          const float s = y[i][k];
          y[i][k] = z[i];
          z[i] = s;
        }
        if (++index >= stretch_) {
          index = 0;
        }
      }
      chain_rotated_ = true;
    } else if (chain_rotated_) {
      UnrotateChain();
    }

    const float a0 = dispersion_[0];
    const float a1 = dispersion_[1];
    const float a2 = dispersion_[2];
    const float a3 = dispersion_[3];
    for (size_t k = 0; a0 != 0.0f && k < size; ++k) {
      float s0 = y[0][k];
      float s1 = y[1][k];
      float s2 = y[2][k];
      float s3 = y[3][k];
      for (size_t stage = 0; stage < size_t(num_stages_); ++stage) {
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

      // Even lines take the left input, odd lines the right one.
      const float xl = tank_in[0][k];
      const float xr = tank_in[1][k];
      for (size_t i = 0; i < kReverbNumLines; i += 2) {
        Write(line_[i], t, s[i] + input_gain_[i] * xl);
        Write(line_[i + 1], t, s[i + 1] + input_gain_[i + 1] * xr);
      }

      l += early_l[k] * early_level_;
      r += early_r[k] * early_level_;
      l = output_lp_[0].Process<stmlib::FILTER_MODE_LOW_PASS>(l);
      r = output_lp_[1].Process<stmlib::FILTER_MODE_LOW_PASS>(r);

      const float wet = wet_modulation.Next();
      wet_out[k].l = l * wet;
      wet_out[k].r = r * wet;
    }
    std::copy(&state[0], &state[kReverbNumLines], &absorption_state_[0]);
    write_ptr_ = t;
  }

  uint16_t* pool_[kReverbNumPools];
  uint32_t used_[kReverbNumPools];
  uint16_t small_pool_[kReverbPoolSize[1]];
  uint16_t smaller_pool_[kReverbPoolSize[2]];

  DelayLine line_[kReverbNumLines];
  DelayLine pre_delay_line_[2];
  DelayLine diffuser_line_[2][kReverbNumDiffusers];
  int32_t diffuser_delay_[kReverbNumDiffusers];
  DelayLine wash_line_[2][kReverbNumWashDiffusers];
  int32_t wash_delay_[2][kReverbNumWashDiffusers];

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
  int32_t num_stages_;
  int32_t chain_index_;
  float dispersion_[kReverbNumSpringLines];
  float chain_[kReverbNumSpringStages][kReverbMaxSpringStretch][
      kReverbNumSpringLines];
  // While there is no dispersion, the stage of a slot is rotated by this
  // offset: stage k lives at row (k + offset) % num_stages_.
  int32_t chain_offset_[kReverbMaxSpringStretch];
  bool chain_rotated_;

  float pre_delay_;
  float pre_delay_increment_;
  float early_delay_[2][kReverbNumEarlyTaps];
  float early_increment_[2][kReverbNumEarlyTaps];
  float early_level_;
  float early_feed_;
  float diffuser_gain_;
  float dry_wash_;
  float wet_wash_;
  float wash_gain_;
  float duck_envelope_;

  // 1 or 2: the rate of the tank is the rate of the module over factor_.
  size_t factor_;
  float decimator_history_[2][kReverbDecimatorHistory];
  float interpolator_history_[2][kReverbInterpolatorHistory];
  int32_t quiet_samples_;
  int32_t quiet_threshold_;

  stmlib::OnePole input_hp_[2];
  stmlib::Svf input_lp_[2];
  stmlib::OnePole output_lp_[2];

  DISALLOW_COPY_AND_ASSIGN(MorphReverb);
};

}  // namespace beads

#endif  // BEADS_DSP_FX_MORPH_REVERB_H_
