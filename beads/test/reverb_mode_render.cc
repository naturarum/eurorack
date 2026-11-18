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
// Renders the morphing reverb and the original Beads reverb on test material,
// and prints decay times, loudness and host CPU time.
//
//
// Renders test material through the reverb-only start mode, and through the
// reverb of the granular mode at the same settings, to compare them by ear.
// Also checks the memory use, the noise floor and the host CPU time.
//
// Build and run with beads/test/render_reverb_mode.sh.

// The signal generators and the WAV code of the other render tool.
#define main reverb_render_main
#include "beads/test/reverb_render.cc"
#undef main

#include "beads/dsp/reverb_mode.h"

const size_t kReverbModeBufferSize = 131072;  // 512 KB of float samples.
float reverb_mode_buffer[kReverbModeBufferSize];
ReverbMode reverb_mode;

Signal RenderReverbMode(
    const Signal& in, const ReverbParameters& p, float dry_wet,
    float pre_delay = 0.0f) {
  reverb_mode.Init(reverb_mode_buffer, kReverbModeBufferSize, kSampleRate);
  ReverbMode::Parameters parameters;
  parameters.reverb = p;
  parameters.dry_wet = dry_wet;
  parameters.pre_delay = pre_delay;
  Signal out(in.size());
  for (size_t i = 0; i < in.size(); i += kBlockSize) {
    size_t n = min(kBlockSize, in.size() - i);
    reverb_mode.Process(parameters, &in[i], &out[i], n);
  }
  return out;
}

// The same path in granular mode: DRY/WET at 0, REVERB knob up, between the
// two limiters of the processor.
Signal RenderGranularModeReverb(
    const Signal& in, const ReverbParameters& p, float knob) {
  Signal scaled = in;
  for (size_t i = 0; i < scaled.size(); ++i) {
    scaled[i].l *= 0.75f;
    scaled[i].r *= 0.75f;
  }
  Limiter pre_reverb_limiter;
  Limiter output_limiter;
  pre_reverb_limiter.Init(1.0f / (kSampleRate * 4.0f), 0.9f);
  output_limiter.Init(1.0f / (kSampleRate * 6.0f), 0.9f);
  pre_reverb_limiter.Process(&scaled[0], scaled.size());
  Signal out = RenderMorph(scaled, kSampleRate, knob, p);
  output_limiter.Process(&out[0], out.size());
  return out;
}

Signal Scale(const Signal& in, float gain) {
  Signal out = in;
  for (size_t i = 0; i < out.size(); ++i) {
    out[i].l *= gain;
    out[i].r *= gain;
  }
  return out;
}

// RMS level of both channels over a window, in dB.
float LevelDb(const Signal& s, float from_s, float to_s) {
  double sum = 0.0;
  size_t n = 0;
  for (size_t i = size_t(from_s * kSampleRate);
       i < size_t(to_s * kSampleRate) && i < s.size(); ++i) {
    sum += double(s[i].l) * s[i].l + double(s[i].r) * s[i].r;
    n += 2;
  }
  return n && sum > 0.0 ? 10.0f * log10f(float(sum / n)) : -200.0f;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <output directory>\n", argv[0]);
    return 1;
  }
  string dir = argv[1];
  srand(1);
  int errors = 0;

  reverb_mode.Init(reverb_mode_buffer, kReverbModeBufferSize, kSampleRate);
  printf("delay memory: %u of %zu samples (%u KB)\n",
         reverb_mode.memory_used(), kReverbModeBufferSize,
         reverb_mode.memory_used() * 4 / 1024);
  if (reverb_mode.memory_used() > kReverbModeBufferSize) {
    fprintf(stderr, "delay memory overflow\n");
    ++errors;
  }

  const Setting settings[] = {
    { "01_room", Preset(0.0f) },
    { "02_hall", Preset(0.5f) },
    { "03_spring", Preset(1.0f) },
    { "04_hall_huge_long", Preset(0.5f, 0.95f, 0.8f) },
    { "05_room_tiny", Preset(0.0f, 0.1f, 0.3f) },
    { "06_hall_no_diffusion_echoes", Preset(0.5f, 0.6f, 0.5f, 0.55f, 0.0f) },
    { "07_hall_bright", Preset(0.5f, 0.6f, 0.55f, 0.95f) },
    { "08_hall_wash", Preset(0.5f, 0.6f, 0.55f, 0.55f, 1.0f) },
    { "09_room_to_hall", Preset(0.25f) },
    { "10_hall_to_spring", Preset(0.75f) },
  };
  const size_t num_settings = sizeof(settings) / sizeof(settings[0]);

  // Demo material, at a balanced mix: the same knob position in both modes.
  Signal demo = MakeDemo();
  const float knob = 0.75f;
  for (size_t i = 0; i < num_settings; ++i) {
    Signal full = RenderReverbMode(demo, settings[i].p, knob);
    Signal budget = RenderGranularModeReverb(demo, settings[i].p, knob);
    float peak = PeakAndCheck(full, settings[i].name);
    if (!(peak < 1.0f)) {
      printf("warning: %s peaks at %.2f\n", settings[i].name, peak);
    }
    WriteWav(dir + "/" + settings[i].name + "_reverb_mode.wav", full,
             kSampleRate);
    WriteWav(dir + "/" + settings[i].name + "_granular_mode.wav", budget,
             kSampleRate);
    // Levels of the wet signal alone, on quiet material that leaves the
    // limiters idle.
    Signal quiet = Scale(demo, 0.25f);
    printf("%-30s wet level, reverb mode %6.1f dB, granular mode %6.1f dB\n",
           settings[i].name,
           LevelDb(RenderReverbMode(quiet, settings[i].p, 1.0f), 0.0f, 21.0f),
           LevelDb(RenderGranularModeReverb(quiet, settings[i].p, 1.0f),
                   0.0f, 21.0f));
  }

  // Fully wet, and the extra pre-delay.
  WriteWav(dir + "/11_hall_fully_wet_reverb_mode.wav",
           RenderReverbMode(demo, Preset(0.5f), 1.0f), kSampleRate);
  WriteWav(dir + "/12_hall_pre_delay_120ms_reverb_mode.wav",
           RenderReverbMode(demo, Preset(0.5f), knob, 0.7f), kSampleRate);

  // DRY/WET at 0 passes the input at the dry level of the granular mode.
  {
    // Quiet, to stay below the limiter. The dry level fades in over the
    // first block.
    Signal quiet = Scale(demo, 0.25f);
    Signal out = RenderReverbMode(quiet, Preset(0.5f), 0.0f);
    float worst = 0.0f;
    for (size_t i = kBlockSize; i < out.size(); ++i) {
      worst = max(worst, fabsf(out[i].l - 0.75f * quiet[i].l));
      worst = max(worst, fabsf(out[i].r - 0.75f * quiet[i].r));
    }
    printf("DRY/WET at 0: largest difference with the input %.2g\n", worst);
    if (worst > 1e-6f) {
      ++errors;
    }
  }

  // Fully wet: nothing of the input before the first reflection.
  {
    Signal out = RenderReverbMode(MakeClick(1.0f), Preset(0.5f), 1.0f);
    const size_t click = size_t(kSampleRate * 0.05f);
    printf("fully wet: output at the click %.2g\n", fabsf(out[click].l));
    if (fabsf(out[click].l) > 1e-6f) {
      ++errors;
    }
  }

  // Noise floor: a loud burst into a long hall, then the level of the tail
  // against the level left after 60 dB more of decay time.
  {
    ReverbParameters p = Preset(0.5f, 0.5f, 0.35f);
    Signal probe = MakeProbe(14.0f);
    Signal full = RenderReverbMode(probe, p, 1.0f);
    Signal budget = RenderGranularModeReverb(probe, p, 1.0f);
    printf("tail of a short hall, fully wet (dB):\n");
    printf("  %-14s %10s %10s\n", "window", "reverb", "granular");
    const float windows[][2] = {
      { 0.1f, 0.5f }, { 1.0f, 1.5f }, { 3.0f, 3.5f }, { 6.0f, 6.5f },
      { 12.0f, 13.0f }
    };
    for (size_t i = 0; i < 5; ++i) {
      printf("  %4.1f - %4.1f s  %10.1f %10.1f\n",
             windows[i][0], windows[i][1],
             LevelDb(full, windows[i][0], windows[i][1]),
             LevelDb(budget, windows[i][0], windows[i][1]));
    }
  }

  // All the controls jump to random positions every 50 ms, as fast CVs
  // would move them: the output must stay finite and within the limiter.
  {
    Signal loop = Loop(demo, 3);
    reverb_mode.Init(reverb_mode_buffer, kReverbModeBufferSize, kSampleRate);
    ReverbMode::Parameters parameters;
    Signal out(loop.size());
    float peak = 0.0f;
    bool finite = true;
    for (size_t i = 0; i < loop.size(); i += kBlockSize) {
      if (i % 2400 == 0) {
        parameters.reverb.character = Noise() * 0.5f + 0.5f;
        parameters.reverb.size = Noise() * 0.5f + 0.5f;
        parameters.reverb.decay = Noise() * 0.5f + 0.5f;
        parameters.reverb.tone = Noise() * 0.5f + 0.5f;
        parameters.reverb.diffusion = Noise() * 0.5f + 0.5f;
        parameters.reverb.ducking = Noise() * 0.5f + 0.5f;
        parameters.dry_wet = Noise() * 0.5f + 0.5f;
        parameters.pre_delay = Noise() * 0.5f + 0.5f;
      }
      size_t n = min(kBlockSize, loop.size() - i);
      reverb_mode.Process(parameters, &loop[i], &out[i], n);
      for (size_t k = 0; k < n; ++k) {
        finite = finite && isfinite(out[i + k].l) && isfinite(out[i + k].r);
        peak = max(peak, max(fabsf(out[i + k].l), fabsf(out[i + k].r)));
      }
    }
    printf("random controls: peak %.2f, %s\n", peak,
           finite ? "finite" : "NOT FINITE");
    if (!finite || peak > 2.0f) {
      ++errors;
    }
    WriteWav(dir + "/13_random_controls_reverb_mode.wav", out, kSampleRate);
  }

  // Host CPU time, relative to real time.
  {
    Signal loop = Loop(demo, 8);
    const ReverbParameters p[3] = {
      Preset(0.5f), Preset(1.0f), Preset(0.5f, 0.6f, 0.55f, 0.55f, 1.0f)
    };
    const char* name[3] = { "hall", "spring", "hall wash" };
    for (int i = 0; i < 3; ++i) {
      auto start = chrono::steady_clock::now();
      Signal full = RenderReverbMode(loop, p[i], knob);
      double t_full = chrono::duration<double>(
          chrono::steady_clock::now() - start).count();
      start = chrono::steady_clock::now();
      Signal budget = RenderGranularModeReverb(loop, p[i], knob);
      double t_budget = chrono::duration<double>(
          chrono::steady_clock::now() - start).count();
      printf("host time, %-10s reverb mode %.3f s, granular mode reverb "
             "%.3f s (x%.1f) for %.0f s of audio\n",
             name[i], t_full, t_budget, t_full / t_budget,
             float(loop.size()) / kSampleRate);
    }
  }

  if (errors) {
    fprintf(stderr, "%d check(s) failed\n", errors);
    return 1;
  }
  printf("OK\n");
  return 0;
}
