// Copyright 2017-2021 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
// Author: Frédéric Meslin (fred@fredslab.net)
// Author: Oliver Rockstedt (info@sourcebox.de)
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

#include <algorithm>

#include "beads/background_buffer_backup.h"
#include "beads/calibrator.h"
#include "beads/cv_reader.h"
#include "beads/drivers/codec.h"
#include "beads/drivers/cycle_counter.h"
#include "beads/drivers/debug_pin.h"
#include "beads/drivers/freeze_input.h"
#include "beads/drivers/seed_input.h"
#include "beads/drivers/system.h"
#include "beads/drivers/watchdog.h"
#include "beads/dsp/processor.h"
#include "beads/factory_test.h"
#include "beads/gain_control.h"
#include "beads/normalization_detector.h"
#include "beads/settings.h"
#include "beads/ui.h"

using namespace std;
using namespace beads;
using namespace stmlib;

const int kBlockRate = 4000;

const bool skip_factory_test = false;

uint8_t delay_buffer_1[512 * 1024] __attribute__ ((section (".ramd1")));
uint8_t delay_buffer_2[256 * 1024] __attribute__ ((section (".ramd2")));
uint16_t reverb_buffer[32768] __attribute__ ((section (".ramd3")));

BackgroundBufferBackup background_buffer_backup;
Calibrator calibrator;
CvReader cv_reader;
Codec codec;
FactoryTest factory_test;
FreezeInput freeze_input;
GainControl gain_control;
NormalizationDetector normalization_detector;
Processor processor;
SeedInput seed_input;
Settings settings;
Ui ui;

volatile bool ready = false;

// Load of the audio interrupt, in CPU cycles per block: 100000 cycles is the
// whole budget (400 MHz, 4000 blocks per second). Read over SWD while the
// module runs, with "make -f beads/makefile cpu_load".
struct CpuLoad {
  uint32_t last;
  uint32_t peak;
};
volatile CpuLoad cpu_load;

class ScopedCpuLoadMeter {
 public:
  ScopedCpuLoadMeter() : start_(CycleCounter::value()) { }
  ~ScopedCpuLoadMeter() {
    uint32_t elapsed = CycleCounter::value() - start_;
    cpu_load.last = elapsed;
    if (elapsed > cpu_load.peak) {
      cpu_load.peak = elapsed;
    }
  }
  
 private:
  uint32_t start_;
  
  DISALLOW_COPY_AND_ASSIGN(ScopedCpuLoadMeter);
};

// #define PROFILE_INTERRUPT

extern "C" {
  
void SysTick_Handler() {
  // Takes about 2µs to complete
  HAL_IncTick();
  Watchdog::Reload();
  if (!ready) {
    return;
  }
  ui.Poll();
  if (!skip_factory_test) {
    factory_test.Poll();
  }
}

}


// This counter is used to temporarily blank the codec output and disable
// the AGC while the normalization detection system is still catching up with
// a codec restart.
int codec_transition_frames;

// Set at boot, until the main loop configures the processor and starts the
// codec for the first time.
bool codec_stopped;

GateFlags previous_seed;
stmlib::DelayLine<Codec::Event, 256> event_delay;

void Process(
    const Codec::Event* events,
    const Codec::Frame* in,
    Codec::Frame* out,
    size_t size) {
#ifdef PROFILE_INTERRUPT
  ScopedDebugPinToggler toggler;
#endif  // PROFILE_INTERRUPT
  ScopedCpuLoadMeter cpu_load_meter;
  FloatFrame float_in[kMaxCodecBlockSize];
  FloatFrame float_out[kMaxCodecBlockSize];
  bool freeze[kMaxCodecBlockSize];
  GateFlags seed[kMaxCodecBlockSize];


  // Normalization detection.
  normalization_detector.Process(
      events[size - 1],
      in[size - 1],
      cv_reader.cv_values());
  
  // CV Reading
  cv_reader.Read();
  for (int i = 0; i < Ui::MIX_PARAMETER_LAST; ++i) {
    ui.ProcessPot(
        Ui::MixParameter(i),
        cv_reader.pot(PotsAdcChannel(POTS_ADC_CHANNEL_FEEDBACK + i)));
  }
  const PotsAdcChannel main_pot_channel[Ui::MAIN_PARAMETER_LAST] = {
    POTS_ADC_CHANNEL_TIME,
    POTS_ADC_CHANNEL_SIZE,
    POTS_ADC_CHANNEL_SHAPE,
    POTS_ADC_CHANNEL_PITCH,
    POTS_ADC_CHANNEL_DENSITY
  };
  for (int i = 0; i < Ui::MAIN_PARAMETER_LAST; ++i) {
    ui.ProcessMainPot(
        Ui::MainParameter(i),
        cv_reader.pot(main_pot_channel[i]));
  }
  
  // Conversion of events into the right format.
  bool freeze_block = false;
  for (size_t i = 0; i < size; ++i) {
    freeze[i] = ui.freeze_button_state() || (events[i] & Codec::EVENT_FREEZE);
    freeze_block |= freeze[i];
  }
  ui.set_freeze_led_state(freeze_block);
  
  bool seed_block = false;
  GateFlags* seed_input = NULL;
  if (!normalization_detector.normalized(NORMALIZATION_DETECTOR_CHANNEL_SEED)) {
    for (size_t i = 0; i < size; ++i) {
      event_delay.Write(events[i]);
      // Delay the seed input by 2.5ms.
      Codec::Event delayed_event = event_delay.Read(size * 10);
      bool has_seed = delayed_event & Codec::EVENT_SEED;
      previous_seed = seed[i] = ExtractGateFlags(previous_seed, has_seed);
      seed_block |= has_seed;
    }
    seed_input = seed;
  } else {
    previous_seed = GATE_FLAG_LOW;
  }
  ui.set_seed_led_state(seed_block && !codec_transition_frames);
  ui.set_overwrite_reserve(processor.overwrite_reserve());
  
  // Conversion of input samples into floats, (with automatic gain control).
  gain_control.Process(
      !codec_transition_frames,
      settings.state().gain,
      normalization_detector.normalized(
          NORMALIZATION_DETECTOR_CHANNEL_AUDIO_LEFT),
      normalization_detector.normalized(
          NORMALIZATION_DETECTOR_CHANNEL_AUDIO_RIGHT),
      in,
      float_in,
      size);
  if (codec_transition_frames) {
    --codec_transition_frames;
    Codec::Frame silence;
    silence.l = 0;
    silence.r = 0;
    fill(&out[0], &out[size], silence);
    return;
  }
  
  Parameters p;
  p.time.base = ui.main_parameter(Ui::MAIN_PARAMETER_TIME);
  p.time.attenuverter = cv_reader.pot(POTS_ADC_CHANNEL_TIME_AMOUNT);
  p.time.has_cv = !normalization_detector.normalized(
      NORMALIZATION_DETECTOR_CHANNEL_TIME_CV);
  p.time.cv = cv_reader.cv(CV_ADC_CHANNEL_TIME);
  
  p.size.base = ui.main_parameter(Ui::MAIN_PARAMETER_SIZE);
  p.size.attenuverter = cv_reader.pot(POTS_ADC_CHANNEL_SIZE_AMOUNT);
  p.size.has_cv = !normalization_detector.normalized(
      NORMALIZATION_DETECTOR_CHANNEL_SIZE_CV);
  p.size.cv = cv_reader.cv(CV_ADC_CHANNEL_SIZE);

  p.shape.base = ui.main_parameter(Ui::MAIN_PARAMETER_SHAPE);
  p.shape.attenuverter = cv_reader.pot(POTS_ADC_CHANNEL_SHAPE_AMOUNT);
  p.shape.has_cv = !normalization_detector.normalized(
      NORMALIZATION_DETECTOR_CHANNEL_SHAPE_CV);
  p.shape.cv = cv_reader.cv(CV_ADC_CHANNEL_SHAPE);
  
  p.pitch.base = PotChannel::Detune(
      ui.main_parameter(Ui::MAIN_PARAMETER_PITCH));
  p.pitch.attenuverter = cv_reader.pot(POTS_ADC_CHANNEL_PITCH_AMOUNT);
  p.pitch.has_cv = !normalization_detector.normalized(
      NORMALIZATION_DETECTOR_CHANNEL_PITCH_CV);
  p.pitch.cv = cv_reader.cv(CV_ADC_CHANNEL_PITCH);
  
#ifdef ENABLE_ADVANCED_FEATURES
  p.monophonic_mode = settings.state().monophonic_mode;
#else
  p.monophonic_mode = false;
#endif  // ENABLE_ADVANCED_FEATURES

  ModulatedParameter d;
  d.base = ui.main_parameter(Ui::MAIN_PARAMETER_DENSITY);
  d.cv = cv_reader.cv(CV_ADC_CHANNEL_DENSITY);
  if (normalization_detector.normalized(
      NORMALIZATION_DETECTOR_CHANNEL_DENSITY_CV)) {
    // Avoid small but annoying modulation due to noise on the density
    // CV input.
    d.cv = 0.0f;
  }

  float mix_parameter[Ui::MIX_PARAMETER_LAST];
  for (int i = 0; i < Ui::MIX_PARAMETER_LAST; ++i) {
    float base = ui.mix_parameter(Ui::MixParameter(i)) * 1.005f;
    float amount = settings.state().assignable_cv_destination == 3
        ? float(settings.state().assignable_cv_amount[i]) / 256.0f
        : (settings.state().assignable_cv_destination == i ? 1.0f : 0.0f);
    mix_parameter[i] = base + \
        (cv_reader.cv(CV_ADC_CHANNEL_ASSIGN) - 0.002f) * amount;
    CONSTRAIN(mix_parameter[i], 0.0f, 1.0f);
  }

  Processor::MixParameters m;
  m.feedback = mix_parameter[Ui::MIX_PARAMETER_FEEDBACK];
  m.dry_wet = mix_parameter[Ui::MIX_PARAMETER_DRY_WET];
  m.reverb_amount = mix_parameter[Ui::MIX_PARAMETER_REVERB];
  m.output_gate = settings.state().gate_output_mode;
  
  const uint8_t* reverb = settings.state().reverb;
  m.reverb.character = float(reverb[REVERB_SETTING_CHARACTER]) / 255.0f;
  m.reverb.size = float(reverb[REVERB_SETTING_SIZE]) / 255.0f;
  m.reverb.decay = float(reverb[REVERB_SETTING_DECAY]) / 255.0f;
  m.reverb.tone = float(reverb[REVERB_SETTING_TONE]) / 255.0f;
  m.reverb.diffusion = float(reverb[REVERB_SETTING_DIFFUSION]) / 255.0f;

  SeedButtonState seed_button_state = ui.seed_button_locked()
      ? SEED_BUTTON_STATE_LOCKED
      : (ui.seed_button_state() ? SEED_BUTTON_STATE_ON : SEED_BUTTON_STATE_OFF);
  
  if (ui.trim_flag()) {
    processor.Trim();
    ui.ClearTrimFlag();
  }
  processor.Process(
      seed_button_state,
      seed_input,
      freeze,
      d, p, m,
      float_in,
      float_out, size);
  
  // Conversion of output samples into integers.
  for (size_t i = 0; i < size; ++i) {
    float l = float_out[i].l;
    float r = float_out[i].r;
    CONSTRAIN(l, -1.0f, 0.999999f);
    CONSTRAIN(r, -1.0f, 0.999999f);
    out[i].l = static_cast<int32_t>(l * 2147483648.0f);
    out[i].r = static_cast<int32_t>(r * 2147483648.0f);
  }
}

void ConfigureAndStartCodec(bool reset_gain_control) {
  int sr = int(processor.sample_rate());
  codec_stopped = false;

  gain_control.Init(
      float(sr),
      processor.media() == Processor::MEDIA_DIRTY_TAPE
          ? GainControl::SOFT_CLIPPING_MODE_ON
          : (processor.media() == Processor::MEDIA_CLEAN_TAPE
                ? GainControl::SOFT_CLIPPING_MODE_DYNAMIC
                : GainControl::SOFT_CLIPPING_MODE_OFF),
      reset_gain_control);
  normalization_detector.Init(sr == 48000 ? 2 : (sr == 32000 ? 3 : 4));
  event_delay.Init();

  codec_transition_frames = kBlockRate / 4;
  codec.Start(sr, sr / kBlockRate, &Process);
}

void Init() {
  System sys;
  sys.Init(true);

  Watchdog::Start();
  CycleCounter::Init();
  freeze_input.Init();
  seed_input.Init();
  codec.Init();

  cv_reader.Init(&settings);
  calibrator.Init(&settings, &cv_reader, &normalization_detector);
  
  uint8_t* delay_buffers[3];
  delay_buffers[0] = delay_buffer_1;
  delay_buffers[1] = delay_buffer_1 + (256 * 1024);
  delay_buffers[2] = delay_buffer_2;
  
  processor.Init(delay_buffers, reverb_buffer);
  background_buffer_backup.Init(&processor, &settings);

  ui.Init(&settings, &gain_control, &calibrator, &background_buffer_backup);
  
  factory_test.Init(
      &cv_reader,
      &normalization_detector,
      &calibrator,
      &ui.switches());
  
  gain_control.Init(48000.0f, GainControl::SOFT_CLIPPING_MODE_OFF, true);

  const bool recovery_mode = ui.pressed_immediate(SWITCH_SEED);
  const bool freshly_baked = !settings.Init(recovery_mode);
  if (freshly_baked && !skip_factory_test) {
    factory_test.Start();
    ui.set_factory_test(true);
    normalization_detector.Init(2);
    codec.Start(48000, 48000 / kBlockRate, &FactoryTest::ProcessFn);
  } else {
#ifdef PROFILE_INTERRUPT
    DebugPin::Init();
#endif  // PROFILE_INTERRUPT
    ui.set_factory_test(false);
    codec_stopped = true;
  }
  ready = true;
  
  bool was_frozen = background_buffer_backup.Load();
  if (was_frozen) {
    ui.set_freeze_button_state(was_frozen);
    ConfigureAndStartCodec(true);
  }
}

void MonitorStateChange() {
  if (ui.should_ignore_state_change() || codec_transition_frames) {
    return;
  }
  
  // With nothing patched in the audio inputs, the mono detection means
  // nothing: keep the current configuration until a source is plugged in.
  if (normalization_detector.no_input()) {
    return;
  }

  // Awful block of code to troubleshoot glitches related to changes of
  // sample rate and more general codec settings.
  //
  // static uint32_t elapsed = 0;
  // bool autoreset = (system_clock.milliseconds() - elapsed) >= 400;
  // if (autoreset) {
  //   settings.mutable_state()->media =
  //       (settings.mutable_state()->media + 1) % 4;
  // }

  Processor::Media media = Processor::Media(settings.state().media);
  bool mono = normalization_detector.mono();
  bool media_changed = media != processor.media();
  bool mono_changed = mono != processor.mono();

  if (codec_stopped || mono_changed || media_changed) {
    // elapsed = system_clock.milliseconds();
    codec.Stop();
    processor.Configure(media, mono);
    ConfigureAndStartCodec(mono_changed || codec_stopped);
  }
}

int main(void) {
  Init();

  while (1) {
    ui.DoEvents();
    MonitorStateChange();
    if (!codec_stopped) {
      background_buffer_backup.Process(ui.freeze_led_state());
    }
    processor.WasteCPUCycles(16);
  }
}

