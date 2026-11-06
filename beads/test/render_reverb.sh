#!/bin/sh
# Builds the reverb render tool natively and renders the test material.
# Output goes to build/reverb_render/. Pass a 16 or 24-bit WAV file to also
# render it through the five main characters.
set -e
cd "$(dirname "$0")/../.."
OUT=build/reverb_render
mkdir -p "$OUT"
c++ -std=c++14 -DTEST -O2 -g -Wall -Wno-unused-local-typedef \
  -Wno-vla-cxx-extension -I. \
  beads/test/reverb_render.cc \
  beads/dsp/processor.cc \
  beads/dsp/mu_law.cc \
  beads/dsp/delay_clock.cc \
  beads/dsp/delay_controller.cc \
  beads/dsp/trigger_generator.cc \
  beads/resources.cc \
  stmlib/utils/random.cc \
  stmlib/dsp/units.cc \
  -o "$OUT/reverb_render"
"$OUT/reverb_render" "$OUT" "$@"
