// Copyright 2015 Emilie Gillet.
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
// Handles the "hold button to tweak a hidden parameter" behaviour of pots.
// A pot can be in 4 states:
// - POT_STATE_TRACKING: the main parameter tracks the position of the pot.
// - POT_STATE_LOCKING: the main parameter no longer tracks the position of
//   the pot. We wait for the pot to move further from its original position
//   to start modifying the hidden parameter.
// - POT_STATE_HIDDEN_PARAMETER: the hidden parameter tracks the position of
//   the pot.
// - POT_STATE_CATCHING_UP: the main parameter stays frozen at the value it
//   had before the hidden parameter was edited, until the pot reaches or
//   crosses that position. The pot then tracks again, and caught_up() reports
//   it once so that the UI can flash a LED. (The stock firmware adjusted the
//   main parameter in a relative way instead, which changed the patch.)

#ifndef BEADS_POT_CONTROLLER_H_
#define BEADS_POT_CONTROLLER_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"

namespace beads {

enum PotState {
  POT_STATE_TRACKING,
  POT_STATE_LOCKING,
  POT_STATE_HIDDEN_PARAMETER,
  POT_STATE_CATCHING_UP
};
  
class PotController {
 public:
  PotController() { }
  ~PotController() { }
  
  inline void Init(
      float scale,
      float offset) {
    state_ = POT_STATE_TRACKING;
    was_catching_up_ = false;
    caught_up_ = false;

    value_ = 0.0f;
    stored_value_ = 0.0f;
    previous_value_ = 0.0f;
    scale_ = scale;
    offset_ = offset;
  }
  
  inline void Lock() {
    if (state_ == POT_STATE_LOCKING || state_ == POT_STATE_HIDDEN_PARAMETER) {
      return;
    }
    was_catching_up_ = state_ == POT_STATE_CATCHING_UP;
    state_ = POT_STATE_LOCKING;
  }
  
  inline float main_parameter() const { return main_parameter_; }
  inline float hidden_parameter() const { return hidden_parameter_; }
  
  inline bool editing_hidden_parameter() const {
    return state_ == POT_STATE_HIDDEN_PARAMETER;
  }
  
  // True while the main parameter is frozen, waiting for the pot to come
  // back to it.
  inline bool catching_up() const {
    return state_ == POT_STATE_CATCHING_UP;
  }
  
  // True once, when the pot has just picked its main parameter up again.
  inline bool caught_up() {
    bool flag = caught_up_;
    caught_up_ = false;
    return flag;
  }
  
  inline void Unlock() {
    if (state_ == POT_STATE_HIDDEN_PARAMETER ||
        (state_ == POT_STATE_LOCKING && was_catching_up_)) {
      state_ = POT_STATE_CATCHING_UP;
    } else if (state_ == POT_STATE_LOCKING) {
      state_ = POT_STATE_TRACKING;
    }
  }
  
  inline void Realign() {
    state_ = POT_STATE_TRACKING;
  }
  
  inline void ProcessControlRate(float adc_value) {
    value_ = adc_value;
    if (state_ == POT_STATE_TRACKING) {
      main_parameter_ = value_;
    }
  }
  
  inline void ProcessUIRate() {
    switch (state_) {
      case POT_STATE_TRACKING:
        previous_value_ = value_;
        break;
        
      case POT_STATE_LOCKING:
        if (fabsf(value_ - previous_value_) > 0.03f) {
          // A pot that was still catching up keeps the value it is frozen at.
          if (!was_catching_up_) {
            stored_value_ = previous_value_;
          }
          hidden_parameter_ = value_;
          state_ = POT_STATE_HIDDEN_PARAMETER;
          previous_value_ = value_;
        }
        break;
      
      case POT_STATE_HIDDEN_PARAMETER:
        hidden_parameter_ = value_;
        previous_value_ = value_;
        break;
        
      case POT_STATE_CATCHING_UP:
        {
          // The main parameter stays where it was before the hidden parameter
          // was edited, until the pot reaches or crosses that position.
          float value = value_;
          CONSTRAIN(value, 0.0f, 1.0f);
          const float error = value - stored_value_;
          const float previous_error = previous_value_ - stored_value_;
          if (fabsf(error) < 0.005f || error * previous_error < 0.0f) {
            state_ = POT_STATE_TRACKING;
            caught_up_ = true;
          }
          previous_value_ = value;
          main_parameter_ = stored_value_ * scale_ + offset_;
        }
        break;
    }
  }
  
 private:
  PotState state_;
  bool was_catching_up_;
  bool caught_up_;
  
  float main_parameter_;
  float hidden_parameter_;
  float value_;
  float stored_value_;
  float previous_value_;
  float scale_;
  float offset_;
  
  DISALLOW_COPY_AND_ASSIGN(PotController);
};

}  // namespace beads

#endif  // BEADS_POT_CONTROLLER_H_
