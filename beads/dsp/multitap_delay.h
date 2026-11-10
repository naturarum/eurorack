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
// Multitap delay.
//
// Modified in 2026 by naturarum: each tap has a pan position and a level in
// the feedback path, which is rendered separately from the output.

#ifndef BEADS_DSP_MULTITAP_DELAY_H_
#define BEADS_DSP_MULTITAP_DELAY_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/filter.h"

#include "beads/dsp/tap.h"
#include "beads/dsp/delay_controller.h"

#define ENABLE_BANDWIDTH_CONTROL

namespace beads {

class MultitapDelay {
 public:
  MultitapDelay() { }
  ~MultitapDelay() { }
  
  inline void Init(float sample_rate) {
    Reset();
    for (int i = 0; i < 4; ++i) {
      filter_[i].Init();
    }
  }
  
  inline void Reset() {
    for (int i = 0; i < DelayController::NUM_TAPS; ++i) {
      tap_[i].Init();
      no_latency_tap_[i].Init();
    }
  }
  
  template<typename DelayLineType>
  inline void Render(
      const DelayLineType* delay_line,
      const DelayController::Output& taps,
      float speed,
      float bandwidth,
      FloatFrame* out,
      FloatFrame* feedback) {
    
    // Cross-fade between the original and pitch-shifted delays.
    float shift = 100.0f * (fabsf(speed - 1.0f) - 0.01f);
    CONSTRAIN(shift, 0.0f, 1.0f);

    FloatFrame mix;
    mix.l = mix.r = 0.0f;
    FloatFrame fb;
    fb.l = fb.r = 0.0f;
    for (int i = 0; i < DelayController::NUM_TAPS; ++i) {
      const DelayTap& t = taps.tap[i];
      if (t.delay == 0) {
        continue;
      }
      FloatFrame f;
      if (shift == 0.0f) {
        f = no_latency_tap_[i].Read(delay_line, t.delay, speed);
      } else if (shift == 1.0f) {
        f = tap_[i].Read(delay_line, t.delay, speed);
      } else {
        const FloatFrame shifted = tap_[i].Read(delay_line, t.delay, speed);
        const FloatFrame plain = no_latency_tap_[i].Read(
            delay_line, t.delay, speed);
        f = plain + (shifted - plain) * shift;
      }
      
      // Panning: towards one side, the tap becomes mono, at constant power.
      const float side = fabsf(t.pan);
      const float mono = 0.7071f * (f.l + f.r) * side;
      const float keep = 1.0f - side;
      mix.l += t.gain * (keep * f.l + (t.pan < 0.0f ? mono : 0.0f));
      mix.r += t.gain * (keep * f.r + (t.pan > 0.0f ? mono : 0.0f));
      fb += f * t.feedback;
    }

#ifdef ENABLE_BANDWIDTH_CONTROL
    const float amount = fabsf(2.0f * bandwidth - 1.0f);
    
    // LP: -7 to -2 octaves
    // HP: -8 to -3 octaves
    const float octave = bandwidth < 0.5f
        ? -7.0f + bandwidth * 10.0f
        : -8.0f + (bandwidth - 0.5f) * 10.0f;
    const float f = 0.49f * stmlib::SemitonesToRatio(12.0f * octave);
    const float q = 1.0f + 16.0f * amount * f;
    const float wet_amount_sqrt = amount * (2.0f - amount);
    const float wet_amount = wet_amount_sqrt * (2.0f - wet_amount_sqrt);
    
    filter_[0].set_f_q<stmlib::FREQUENCY_FAST>(f, q);
    filter_[1].set(filter_[0]);
    filter_[2].set(filter_[0]);
    filter_[3].set(filter_[0]);
    
    // The filter is also in the feedback path, so that each repeat is
    // darker (or thinner) than the previous one.
    FloatFrame wet;
    FloatFrame fb_wet;
    if (bandwidth < 0.5f) {
      wet.l = filter_[0].Process<stmlib::FILTER_MODE_LOW_PASS>(mix.l);
      wet.r = filter_[1].Process<stmlib::FILTER_MODE_LOW_PASS>(mix.r);
      fb_wet.l = filter_[2].Process<stmlib::FILTER_MODE_LOW_PASS>(fb.l);
      fb_wet.r = filter_[3].Process<stmlib::FILTER_MODE_LOW_PASS>(fb.r);
    } else {
      wet.l = filter_[0].Process<stmlib::FILTER_MODE_HIGH_PASS>(mix.l);
      wet.r = filter_[1].Process<stmlib::FILTER_MODE_HIGH_PASS>(mix.r);
      fb_wet.l = filter_[2].Process<stmlib::FILTER_MODE_HIGH_PASS>(fb.l);
      fb_wet.r = filter_[3].Process<stmlib::FILTER_MODE_HIGH_PASS>(fb.r);
    }
    *out += mix + (wet - mix) * wet_amount;
    *feedback += fb + (fb_wet - fb) * wet_amount;
#else
    *out += mix;
    *feedback += fb;
#endif  // ENABLE_BANDWIDTH_CONTROL
  }

 private:
  Tap<2048> tap_[DelayController::NUM_TAPS];
  Tap<1> no_latency_tap_[DelayController::NUM_TAPS];
  stmlib::Svf filter_[4];

  DISALLOW_COPY_AND_ASSIGN(MultitapDelay);
};

}  // namespace beads

#endif  // BEADS_DSP_MULTITAP_DELAY_H_
