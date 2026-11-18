#!/bin/sh
# Builds the render tool of the reverb-only start mode and runs it.
# Output goes to build/reverb_mode_render/.
set -e
cd "$(dirname "$0")/../.."
OUT=build/reverb_mode_render
mkdir -p "$OUT"
c++ -std=c++14 -DTEST -O2 -g -Wall -Wno-unused-local-typedef \
  -Wno-vla-cxx-extension -I. \
  beads/test/reverb_mode_render.cc \
  beads/dsp/processor.cc \
  beads/dsp/mu_law.cc \
  beads/dsp/delay_clock.cc \
  beads/dsp/delay_controller.cc \
  beads/dsp/trigger_generator.cc \
  beads/resources.cc \
  stmlib/utils/random.cc \
  stmlib/dsp/units.cc \
  -o "$OUT/reverb_mode_render"
"$OUT/reverb_mode_render" "$OUT" "$@"
