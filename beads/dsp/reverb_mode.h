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
// The whole FX chain.
//
// Reverb-only start mode: the inputs go straight into the full-quality
// reverb, and the granular processor is left alone.

#ifndef BEADS_DSP_REVERB_MODE_H_
#define BEADS_DSP_REVERB_MODE_H_

#include "stmlib/stmlib.h"

#include <algorithm>

#include "beads/dsp/frame.h"
#include "beads/dsp/fx/full_reverb.h"
#include "beads/dsp/limiter.h"

namespace beads {

class ReverbMode {
 public:
  ReverbMode() { }
  ~ReverbMode() { }

  // All in [0, 1].
  struct Parameters {
    ReverbParameters reverb;
    float dry_wet;
    float pre_delay;
  };

  // The buffer holds float samples: its size, in samples, must be a power
  // of two.
  void Init(float* buffer, size_t size, float sample_rate) {
    reverb_.Init(buffer, size, sample_rate);
    output_limiter_.Init(1.0f / (sample_rate * 6.0f), 0.9f);
  }

  inline uint32_t memory_used() const {
    return reverb_.memory_used();
  }

  void Process(
      const Parameters& parameters,
      const FloatFrame* in,
      FloatFrame* out,
      size_t size) {
    // The level of the dry signal in granular mode, with DRY/WET at 0.
    const float kDryGain = 0.75f;
    for (size_t i = 0; i < size; ++i) {
      out[i].l = in[i].l * kDryGain;
      out[i].r = in[i].r * kDryGain;
    }

    // Same curve as the REVERB knob of the granular mode, which stops at
    // 0.9: here the last part of the travel takes the dry signal out.
    const float dry_wet = parameters.dry_wet;
    reverb_.set_amount(dry_wet * dry_wet);
    // More resolution for short pre-delays.
    reverb_.set_extra_pre_delay(
        parameters.pre_delay * parameters.pre_delay * \
            full_reverb::kReverbMaxExtraPreDelayMs);
    reverb_.set_parameters(parameters.reverb);
    reverb_.Process(out, size);

    output_limiter_.Process(out, size);
  }

 private:
  FullReverb reverb_;
  Limiter output_limiter_;

  DISALLOW_COPY_AND_ASSIGN(ReverbMode);
};

}  // namespace beads

#endif  // BEADS_DSP_REVERB_MODE_H_
