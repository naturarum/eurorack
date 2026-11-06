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
// CPU cycle counter of the Cortex-M7 (DWT), for measuring the audio load.

#ifndef BEADS_DRIVERS_CYCLE_COUNTER_H_
#define BEADS_DRIVERS_CYCLE_COUNTER_H_

#include "stmlib/stmlib.h"

#include <stm32h7xx_hal.h>

namespace beads {

class CycleCounter {
 public:
  CycleCounter() { }
  ~CycleCounter() { }

  inline static void Init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // The DWT registers are locked on the Cortex-M7 until this key is written.
    DWT->LAR = 0xc5acce55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  inline static uint32_t value() {
    return DWT->CYCCNT;
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(CycleCounter);
};

}  // namespace beads

#endif  // BEADS_DRIVERS_CYCLE_COUNTER_H_
