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
// Usage: reverb_render <output directory> [input.wav]
// The optional input is a 16 or 24-bit PCM WAV file, mono or stereo.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "beads/dsp/fx/morph_reverb.h"
#include "beads/dsp/fx/reverb.h"
#include "beads/dsp/processor.h"

using namespace beads;
using namespace std;

const float kSampleRate = 48000.0f;
const size_t kBlockSize = 12;

typedef vector<FloatFrame> Signal;

uint16_t reverb_buffer[32768];
uint8_t delay_buffer_1[512 * 1024];
uint8_t delay_buffer_2[256 * 1024];

// --- WAV files ---------------------------------------------------------------

void WriteWav(const string& path, const Signal& s, float sample_rate) {
  FILE* fp = fopen(path.c_str(), "wb");
  if (!fp) {
    fprintf(stderr, "cannot write %s\n", path.c_str());
    exit(1);
  }
  uint32_t data_size = s.size() * 4;
  uint32_t l;
  uint16_t w;
  fwrite("RIFF", 4, 1, fp);
  l = 36 + data_size; fwrite(&l, 4, 1, fp);
  fwrite("WAVEfmt ", 8, 1, fp);
  l = 16; fwrite(&l, 4, 1, fp);
  w = 1; fwrite(&w, 2, 1, fp);
  w = 2; fwrite(&w, 2, 1, fp);
  l = uint32_t(sample_rate); fwrite(&l, 4, 1, fp);
  l = uint32_t(sample_rate) * 4; fwrite(&l, 4, 1, fp);
  w = 4; fwrite(&w, 2, 1, fp);
  w = 16; fwrite(&w, 2, 1, fp);
  fwrite("data", 4, 1, fp);
  fwrite(&data_size, 4, 1, fp);
  for (size_t i = 0; i < s.size(); ++i) {
    float x[2] = { s[i].l, s[i].r };
    for (int c = 0; c < 2; ++c) {
      float v = x[c] * 32767.0f;
      v = v > 32767.0f ? 32767.0f : (v < -32767.0f ? -32767.0f : v);
      int16_t sample = int16_t(lrintf(v));
      fwrite(&sample, 2, 1, fp);
    }
  }
  fclose(fp);
}

bool ReadWav(const string& path, Signal* s, float* sample_rate) {
  FILE* fp = fopen(path.c_str(), "rb");
  if (!fp) {
    return false;
  }
  char id[4];
  uint32_t size;
  if (fread(id, 4, 1, fp) != 1 || string(id, 4) != "RIFF") {
    fclose(fp);
    return false;
  }
  fseek(fp, 12, SEEK_SET);
  uint16_t format = 0, channels = 0, bits = 0;
  uint32_t rate = 0;
  while (fread(id, 4, 1, fp) == 1 && fread(&size, 4, 1, fp) == 1) {
    string chunk(id, 4);
    if (chunk == "fmt ") {
      vector<uint8_t> f(size);
      if (fread(f.data(), size, 1, fp) != 1) break;
      format = f[0] | (f[1] << 8);
      channels = f[2] | (f[3] << 8);
      rate = f[4] | (f[5] << 8) | (f[6] << 16) | (f[7] << 24);
      bits = f[14] | (f[15] << 8);
    } else if (chunk == "data") {
      if (format != 1 || (bits != 16 && bits != 24) || channels < 1 ||
          channels > 2) {
        fprintf(stderr, "only 16 or 24-bit PCM, mono or stereo\n");
        fclose(fp);
        return false;
      }
      size_t bytes = bits / 8;
      size_t frames = size / (bytes * channels);
      vector<uint8_t> data(size);
      if (fread(data.data(), size, 1, fp) != 1) break;
      s->resize(frames);
      for (size_t i = 0; i < frames; ++i) {
        float x[2];
        for (size_t c = 0; c < channels; ++c) {
          const uint8_t* p = &data[(i * channels + c) * bytes];
          int32_t v = bytes == 2
              ? int16_t(p[0] | (p[1] << 8))
              : (int32_t(p[0] << 8 | p[1] << 16 | p[2] << 24) >> 8);
          x[c] = float(v) / (bytes == 2 ? 32768.0f : 8388608.0f);
        }
        (*s)[i].l = x[0];
        (*s)[i].r = channels == 2 ? x[1] : x[0];
      }
      *sample_rate = float(rate);
      fclose(fp);
      return true;
    } else {
      fseek(fp, size + (size & 1), SEEK_CUR);
    }
  }
  fclose(fp);
  return false;
}

// --- Test material -----------------------------------------------------------

float Noise() {
  return float(rand()) / float(RAND_MAX) * 2.0f - 1.0f;
}

void AddKick(Signal* s, size_t start, float gain) {
  float phase = 0.0f;
  for (size_t i = 0; i < 24000 && start + i < s->size(); ++i) {
    float t = float(i) / kSampleRate;
    float f = 45.0f + 90.0f * expf(-t * 30.0f);
    phase += f / kSampleRate;
    float v = sinf(2.0f * float(M_PI) * phase) * expf(-t * 7.0f) * gain;
    (*s)[start + i].l += v;
    (*s)[start + i].r += v;
  }
}

void AddSnare(Signal* s, size_t start, float gain) {
  float lp = 0.0f, hp = 0.0f;
  for (size_t i = 0; i < 12000 && start + i < s->size(); ++i) {
    float t = float(i) / kSampleRate;
    float n = Noise();
    lp += 0.5f * (n - lp);
    hp = n - lp;
    float body = sinf(2.0f * float(M_PI) * 185.0f * t) * expf(-t * 25.0f);
    float v = (0.7f * hp * expf(-t * 18.0f) + 0.5f * body) * gain;
    (*s)[start + i].l += v;
    (*s)[start + i].r += v * 0.9f;
  }
}

void AddHat(Signal* s, size_t start, float gain) {
  float prev = 0.0f;
  for (size_t i = 0; i < 3000 && start + i < s->size(); ++i) {
    float t = float(i) / kSampleRate;
    float n = Noise();
    float v = (n - prev) * expf(-t * 60.0f) * gain;
    prev = n;
    (*s)[start + i].l += v * 0.8f;
    (*s)[start + i].r += v;
  }
}

// Karplus-Strong pluck.
void AddPluck(Signal* s, size_t start, float frequency, float gain, float pan) {
  size_t period = size_t(kSampleRate / frequency);
  vector<float> line(period);
  for (size_t i = 0; i < period; ++i) {
    line[i] = Noise();
  }
  float previous = 0.0f;
  size_t index = 0;
  for (size_t i = 0; i < 72000 && start + i < s->size(); ++i) {
    float x = line[index];
    float y = 0.5f * (x + previous) * 0.996f;
    previous = x;
    line[index] = y;
    index = (index + 1) % period;
    (*s)[start + i].l += x * gain * (1.0f - pan);
    (*s)[start + i].r += x * gain * pan;
  }
}

// A short phrase of drums, then plucks, then a sustained chord, each followed
// by silence so that the tails can be heard.
Signal MakeDemo() {
  Signal s(size_t(kSampleRate * 21.0f));
  for (size_t i = 0; i < s.size(); ++i) {
    s[i].l = s[i].r = 0.0f;
  }
  const size_t sixteenth = size_t(kSampleRate * 60.0f / 100.0f / 4.0f);
  const char* kick =  "x.....x...x.....x.....x...x..x..";
  const char* snare = "....x.......x.......x.......x...";
  const char* hat =   "..x...x...x...x...x...x...x...x.";
  for (size_t step = 0; step < 32; ++step) {
    size_t at = step * sixteenth;
    if (kick[step] == 'x') AddKick(&s, at, 0.7f);
    if (snare[step] == 'x') AddSnare(&s, at, 0.5f);
    if (hat[step] == 'x') AddHat(&s, at, 0.25f);
  }
  const float notes[] = { 220.0f, 261.6f, 329.6f, 392.0f, 440.0f, 392.0f,
                          329.6f, 261.6f, 293.7f, 349.2f, 440.0f, 523.3f };
  const size_t eighth = size_t(kSampleRate * 60.0f / 90.0f / 2.0f);
  size_t start = size_t(kSampleRate * 7.0f);
  for (size_t i = 0; i < 12; ++i) {
    AddPluck(&s, start + i * eighth, notes[i], 0.35f, 0.3f + 0.4f * (i % 2));
  }
  // Sustained chord with a slow attack.
  start = size_t(kSampleRate * 15.0f);
  const float chord[] = { 146.8f, 220.0f, 293.7f, 349.2f };
  float phase[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
  float lp = 0.0f;
  for (size_t i = 0; i < size_t(kSampleRate * 2.0f); ++i) {
    float t = float(i) / kSampleRate;
    float envelope = min(t / 0.3f, 1.0f) * min((2.0f - t) / 0.05f, 1.0f);
    float v = 0.0f;
    for (int n = 0; n < 4; ++n) {
      phase[n] += chord[n] / kSampleRate;
      if (phase[n] >= 1.0f) phase[n] -= 1.0f;
      v += 2.0f * phase[n] - 1.0f;
    }
    lp += 0.08f * (v - lp);
    s[start + i].l += lp * envelope * 0.12f;
    s[start + i].r += lp * envelope * 0.12f;
  }
  return s;
}

Signal MakeClick(float seconds) {
  Signal s(size_t(kSampleRate * seconds));
  for (size_t i = 0; i < s.size(); ++i) {
    s[i].l = s[i].r = 0.0f;
  }
  s[size_t(kSampleRate * 0.05f)].l = 0.8f;
  s[size_t(kSampleRate * 0.05f)].r = 0.8f;
  return s;
}

// A 20 ms burst of noise, a better probe of the tank than a single click:
// the tail starts well above the 16-bit floor of the delay lines.
Signal MakeProbe(float seconds) {
  Signal s(size_t(kSampleRate * seconds));
  for (size_t i = 0; i < s.size(); ++i) {
    s[i].l = s[i].r = 0.0f;
  }
  for (size_t i = 0; i < size_t(kSampleRate * 0.02f); ++i) {
    s[i + size_t(kSampleRate * 0.05f)].l = Noise() * 0.5f;
    s[i + size_t(kSampleRate * 0.05f)].r = Noise() * 0.5f;
  }
  return s;
}

Signal MakeNoiseBurst() {
  Signal s(size_t(kSampleRate * 6.0f));
  float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;
  for (size_t i = 0; i < s.size(); ++i) {
    float v = 0.0f;
    if (i < size_t(kSampleRate * 2.0f)) {
      // Pink-ish noise.
      float n = Noise();
      b0 = 0.99765f * b0 + n * 0.0990460f;
      b1 = 0.96300f * b1 + n * 0.2965164f;
      b2 = 0.57000f * b2 + n * 1.0526913f;
      v = (b0 + b1 + b2 + n * 0.1848f) * 0.08f;
    }
    s[i].l = s[i].r = v;
  }
  return s;
}

// --- Rendering ---------------------------------------------------------------

struct Setting {
  const char* name;
  ReverbParameters p;
};

ReverbParameters Preset(
    float character, float size = 0.5f, float decay = 0.5f,
    float tone = 0.55f, float diffusion = 0.8f) {
  ReverbParameters p;
  p.character = character;
  p.size = size;
  p.decay = decay;
  p.tone = tone;
  p.diffusion = diffusion;
  return p;
}

// The REVERB knob position, as the processor maps it.
float Amount(float knob) {
  return knob * knob * 0.9f;
}

MorphReverb morph;

Signal RenderMorph(
    const Signal& in,
    float sample_rate,
    float knob,
    const ReverbParameters& p,
    ReverbParameters (*automation)(float, const ReverbParameters&) = NULL) {
  morph.Init(reverb_buffer, sample_rate);
  Signal out = in;
  for (size_t i = 0; i < out.size(); i += kBlockSize) {
    size_t n = min(kBlockSize, out.size() - i);
    ReverbParameters q = p;
    if (automation) {
      q = automation(float(i) / float(out.size()), p);
    }
    morph.set_parameters(q);
    morph.set_amount(Amount(knob));
    morph.Process(&out[i], n);
  }
  return out;
}

Reverb stock;

Signal RenderStock(const Signal& in, float knob) {
  stock.Init(reverb_buffer, kSampleRate);
  Signal out = in;
  const float reverb_time = knob * knob * knob;
  for (size_t i = 0; i < out.size(); i += kBlockSize) {
    size_t n = min(kBlockSize, out.size() - i);
    stock.set_mix(Amount(knob));
    stock.set_time(0.3f + 0.6f * reverb_time);
    stock.set_lp(0.6f);
    stock.Process(&out[i], n);
  }
  return out;
}

float PeakAndCheck(const Signal& s, const char* name);

Processor processor;

// The whole Beads processor, dense grains, for a host CPU comparison.
double TimeProcessor(const Signal& in) {
  uint8_t* buffers[3] = {
    delay_buffer_1, delay_buffer_1 + 256 * 1024, delay_buffer_2
  };
  processor.Init(buffers, reverb_buffer);
  processor.Configure(Processor::MEDIA_DIGITAL, false);
  Parameters p;
  RandomizedParameter* parameters[4] = { &p.time, &p.size, &p.shape, &p.pitch };
  const float bases[4] = { 0.4f, 0.75f, 0.5f, 0.0f };
  for (int i = 0; i < 4; ++i) {
    parameters[i]->base = bases[i];
    parameters[i]->attenuverter = 0.0f;
    parameters[i]->cv = 0.0f;
    parameters[i]->has_cv = false;
  }
  p.monophonic_mode = false;
  Processor::MixParameters m;
  m.feedback = 0.3f;
  m.dry_wet = 0.7f;
  m.reverb_amount = 1.0f;
  m.output_gate = false;
  m.reverb.character = 0.75f;
  ModulatedParameter d;
  d.base = 0.95f;
  d.cv = 0.0f;
  bool freeze[kBlockSize];
  fill(&freeze[0], &freeze[kBlockSize], false);
  Signal out(in.size());
  auto start = chrono::steady_clock::now();
  for (size_t i = 0; i + kBlockSize <= in.size(); i += kBlockSize) {
    processor.Process(
        SEED_BUTTON_STATE_OFF, NULL, freeze, d, p, m,
        &in[i], &out[i], kBlockSize);
  }
  double elapsed = chrono::duration<double>(
      chrono::steady_clock::now() - start).count();
  PeakAndCheck(out, "processor");
  return elapsed;
}

// --- Measurements ------------------------------------------------------------

// Decay time from the Schroeder integral (T30, extrapolated to 60 dB).
float MeasureT60(const Signal& ir, size_t start) {
  vector<double> energy(ir.size() - start);
  double sum = 0.0;
  for (size_t i = ir.size(); i-- > start; ) {
    sum += double(ir[i].l) * ir[i].l + double(ir[i].r) * ir[i].r;
    energy[i - start] = sum;
  }
  if (sum <= 0.0) {
    return 0.0f;
  }
  size_t t5 = 0, t35 = 0;
  for (size_t i = 0; i < energy.size(); ++i) {
    double db = 10.0 * log10(energy[i] / energy[0] + 1e-30);
    if (!t5 && db < -5.0) t5 = i;
    if (!t35 && db < -35.0) {
      t35 = i;
      break;
    }
  }
  if (!t35) {
    return -1.0f;  // Did not decay 35 dB within the render.
  }
  return 2.0f * float(t35 - t5) / kSampleRate;
}

// Decay time below about 300 Hz, where the decay control is exact.
float MeasureT60Low(const Signal& ir, size_t start) {
  Signal low = ir;
  float l = 0.0f, r = 0.0f;
  const float coefficient = 2.0f * float(M_PI) * 300.0f / kSampleRate;
  for (size_t i = 0; i < low.size(); ++i) {
    l += coefficient * (low[i].l - l);
    r += coefficient * (low[i].r - r);
    low[i].l = l;
    low[i].r = r;
  }
  return MeasureT60(low, start);
}

// Early to late energy ratio at 50 ms, in dB.
float MeasureC50(const Signal& ir, size_t start) {
  double early = 0.0, late = 0.0;
  size_t split = start + size_t(0.05f * kSampleRate);
  for (size_t i = start; i < ir.size(); ++i) {
    double e = double(ir[i].l) * ir[i].l + double(ir[i].r) * ir[i].r;
    if (i < split) early += e; else late += e;
  }
  return float(10.0 * log10(early / (late + 1e-30) + 1e-30));
}

float RmsDb(const Signal& s) {
  double sum = 0.0;
  for (size_t i = 0; i < s.size(); ++i) {
    sum += double(s[i].l) * s[i].l + double(s[i].r) * s[i].r;
  }
  return float(10.0 * log10(sum / (2.0 * s.size()) + 1e-30));
}

float PeakAndCheck(const Signal& s, const char* name) {
  float peak = 0.0f;
  for (size_t i = 0; i < s.size(); ++i) {
    if (!std::isfinite(s[i].l) || !std::isfinite(s[i].r)) {
      fprintf(stderr, "%s: non-finite sample at %zu\n", name, i);
      exit(1);
    }
    peak = max(peak, max(fabsf(s[i].l), fabsf(s[i].r)));
  }
  return peak;
}

// Inter-channel correlation of the tail, after the first 100 ms.
float Correlation(const Signal& ir, size_t start) {
  double lr = 0.0, ll = 0.0, rr = 0.0;
  for (size_t i = start + size_t(0.1f * kSampleRate); i < ir.size(); ++i) {
    lr += double(ir[i].l) * ir[i].r;
    ll += double(ir[i].l) * ir[i].l;
    rr += double(ir[i].r) * ir[i].r;
  }
  return float(lr / sqrt(ll * rr + 1e-30));
}

// --- Automations for the sweeps -----------------------------------------------

ReverbParameters SweepCharacter(float t, const ReverbParameters& p) {
  ReverbParameters q = p;
  q.character = t < 0.5f ? 2.0f * t : 2.0f - 2.0f * t;
  return q;
}

ReverbParameters SweepSize(float t, const ReverbParameters& p) {
  ReverbParameters q = p;
  q.size = 0.1f + 0.8f * (t < 0.5f ? 2.0f * t : 2.0f - 2.0f * t);
  return q;
}

ReverbParameters SweepDiffusion(float t, const ReverbParameters& p) {
  ReverbParameters q = p;
  q.diffusion = t < 0.5f ? 2.0f * t : 2.0f - 2.0f * t;
  return q;
}

Signal Loop(const Signal& s, size_t times) {
  Signal out;
  for (size_t i = 0; i < times; ++i) {
    out.insert(out.end(), s.begin(), s.end());
  }
  return out;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <output directory> [input.wav]\n", argv[0]);
    return 1;
  }
  string dir = argv[1];
  srand(1);

  morph.Init(reverb_buffer, kSampleRate);
  printf("tank memory at 48 kHz: large pool %u / %u, small pool %u / %u "
         "samples\n",
         morph.memory_used(0), kReverbLargePoolSize,
         morph.memory_used(1), kReverbSmallPoolSize);
  if (morph.memory_used(0) > kReverbLargePoolSize ||
      morph.memory_used(1) > kReverbSmallPoolSize) {
    fprintf(stderr, "delay memory overflow\n");
    return 1;
  }

  const Setting settings[] = {
    { "01_room", Preset(0.0f) },
    { "02_room_to_hall", Preset(0.25f) },
    { "03_hall", Preset(0.5f) },
    { "04_hall_to_spring", Preset(0.75f) },
    { "05_spring", Preset(1.0f) },
    { "06_hall_huge_long", Preset(0.5f, 0.95f, 0.8f) },
    { "07_room_tiny", Preset(0.0f, 0.1f, 0.3f) },
    { "08_spring_separate_springs", Preset(1.0f, 0.5f, 0.5f, 0.55f, 0.15f) },
    { "09_hall_no_diffusion_echoes", Preset(0.5f, 0.6f, 0.5f, 0.55f, 0.0f) },
    { "10_hall_dark", Preset(0.5f, 0.6f, 0.55f, 0.05f) },
    { "11_hall_bright", Preset(0.5f, 0.6f, 0.55f, 0.95f) },
  };
  const size_t num_settings = sizeof(settings) / sizeof(settings[0]);

  // Demo material at REVERB knob 0.75 (a balanced mix).
  Signal demo = MakeDemo();
  {
    Signal out = RenderStock(demo, 0.75f);
    PeakAndCheck(out, "stock");
    WriteWav(dir + "/00_stock_reverb.wav", out, kSampleRate);
  }
  for (size_t i = 0; i < num_settings; ++i) {
    Signal out = RenderMorph(demo, kSampleRate, 0.75f, settings[i].p);
    float peak = PeakAndCheck(out, settings[i].name);
    if (peak > 1.0f) {
      printf("warning: %s peaks at %.2f\n", settings[i].name, peak);
    }
    WriteWav(dir + "/" + settings[i].name + ".wav", out, kSampleRate);
  }

  // Sweeps, on two passes of the demo.
  Signal loop = Loop(demo, 2);
  WriteWav(dir + "/12_sweep_character.wav",
           RenderMorph(loop, kSampleRate, 0.75f, Preset(0.5f), SweepCharacter),
           kSampleRate);
  WriteWav(dir + "/13_sweep_size.wav",
           RenderMorph(loop, kSampleRate, 0.75f, Preset(0.5f), SweepSize),
           kSampleRate);
  WriteWav(dir + "/14_sweep_diffusion.wav",
           RenderMorph(loop, kSampleRate, 0.75f, Preset(0.3f), SweepDiffusion),
           kSampleRate);

  // Impulse responses (REVERB knob at maximum) and measurements.
  Signal click = MakeClick(8.0f);
  Signal probe = MakeProbe(10.0f);
  Signal noise = MakeNoiseBurst();
  const size_t onset = size_t(kSampleRate * 0.05f);
  const size_t probe_end = size_t(kSampleRate * 0.07f);
  printf("\n%-30s %7s %7s %7s %7s %9s %7s\n",
         "setting", "T60 s", "low T60", "C50 dB", "corr", "wet RMS", "peak");
  {
    Signal ir = RenderStock(click, 1.0f);
    Signal tail = RenderStock(probe, 1.0f);
    Signal wet = RenderStock(noise, 1.0f);
    WriteWav(dir + "/ir_00_stock_reverb.wav", ir, kSampleRate);
    printf("%-30s %7.2f %7.2f %7.1f %7.2f %9.1f %7.2f\n", "stock reverb",
           MeasureT60(tail, probe_end), MeasureT60Low(tail, probe_end),
           MeasureC50(ir, onset),
           Correlation(ir, onset), RmsDb(wet), PeakAndCheck(wet, "stock"));
  }
  for (size_t i = 0; i < num_settings; ++i) {
    Signal ir = RenderMorph(click, kSampleRate, 1.0f, settings[i].p);
    Signal tail = RenderMorph(probe, kSampleRate, 1.0f, settings[i].p);
    Signal wet = RenderMorph(noise, kSampleRate, 1.0f, settings[i].p);
    if (i < 5) {
      WriteWav(dir + "/ir_" + settings[i].name + ".wav", ir, kSampleRate);
    }
    printf("%-30s %7.2f %7.2f %7.1f %7.2f %9.1f %7.2f\n", settings[i].name,
           MeasureT60(tail, probe_end), MeasureT60Low(tail, probe_end),
           MeasureC50(ir, onset),
           Correlation(ir, onset), RmsDb(wet), PeakAndCheck(wet, "wet"));
  }

  // Wet-only spring response with separate springs, to check the dispersion.
  {
    morph.Init(reverb_buffer, kSampleRate);
    Signal ir = click;
    for (size_t i = 0; i < ir.size(); i += kBlockSize) {
      morph.set_parameters(Preset(1.0f, 0.5f, 0.5f, 0.55f, 0.15f));
      morph.set_amount(1.0f);
      morph.Process(&ir[i], min(kBlockSize, ir.size() - i));
    }
    WriteWav(dir + "/ir_wet_spring_separate.wav", ir, kSampleRate);
  }

  // Worst case for the tank's int16 storage: long decay, loud input.
  {
    Signal loud = MakeNoiseBurst();
    for (size_t i = 0; i < loud.size(); ++i) {
      loud[i].l *= 4.0f;
      loud[i].r *= 4.0f;
    }
    Signal out = RenderMorph(loud, kSampleRate, 1.0f, Preset(0.5f, 1.0f, 1.0f));
    printf("\nloud input, huge long hall: peak %.2f\n",
           PeakAndCheck(out, "loud"));
  }

  // 32 kHz, as in the vintage digital media.
  {
    Signal ir = RenderMorph(click, 32000.0f, 1.0f, Preset(0.5f));
    PeakAndCheck(ir, "32k");
    printf("hall at 32 kHz: memory %u / %u samples\n",
           morph.memory_used(0), morph.memory_used(1));
  }

  // Host CPU time, reverb alone and the whole Beads processor.
  {
    Signal long_noise = Loop(MakeNoiseBurst(), 10);
    auto start = chrono::steady_clock::now();
    RenderStock(long_noise, 1.0f);
    double stock_time = chrono::duration<double>(
        chrono::steady_clock::now() - start).count();
    start = chrono::steady_clock::now();
    RenderMorph(long_noise, kSampleRate, 1.0f, Preset(0.75f));
    double morph_time = chrono::duration<double>(
        chrono::steady_clock::now() - start).count();
    double seconds = double(long_noise.size()) / kSampleRate;
    double processor_time = TimeProcessor(long_noise);
    printf("\nhost CPU, %.0f s of audio: stock reverb %.1f ms, "
           "morph reverb %.1f ms (%.1fx), whole processor with morph "
           "reverb %.1f ms\n",
           seconds, stock_time * 1000.0, morph_time * 1000.0,
           morph_time / stock_time, processor_time * 1000.0);
  }

  if (argc >= 3) {
    Signal in;
    float sample_rate = kSampleRate;
    if (!ReadWav(argv[2], &in, &sample_rate)) {
      fprintf(stderr, "cannot read %s\n", argv[2]);
      return 1;
    }
    if (sample_rate != kSampleRate) {
      printf("note: %s is at %.0f Hz, rendered at that rate\n",
             argv[2], sample_rate);
    }
    // Two seconds of silence for the tail.
    in.resize(in.size() + size_t(sample_rate * 2.0f), FloatFrame());
    for (size_t i = 0; i < 5; ++i) {
      Signal out = RenderMorph(in, sample_rate, 0.75f, settings[i].p);
      PeakAndCheck(out, settings[i].name);
      WriteWav(dir + "/input_" + settings[i].name + ".wav", out, sample_rate);
    }
  }
  printf("\nwrote renders to %s\n", dir.c_str());
  return 0;
}
