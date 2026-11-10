# Beads with a morphing reverb

Unofficial firmware for Mutable Instruments Beads. The grains sound as on stock Beads. The built-in plate reverb is replaced by one whose character moves continuously from room through hall to spring, and all its settings can be changed from the panel. Version 2 adds a stereo input to the reverb, ducking, a diffusion control that does something across its whole range, and a delay mode with clearer controls.

So far it has been tested on one module. Version 2 is still in testing: until it is released, the release page has version 1.

## What changed

- **A new reverb**, in [`dsp/fx/morph_reverb.h`](dsp/fx/morph_reverb.h):
  - An 8-line feedback delay network, mixed by a matrix built from rotations. Diffusion sets the rotation angles.
  - Four of the lines run through chains of stretched allpass filters, which give the spring its chirp.
  - The spring fades in as you turn towards it; there is no switch.
  - Early reflections and input diffusers sit in front of the network.
  - **Stereo input** (version 2). The left and right sides feed different lines, so grains panned left ring out on the left first. With a single input patched, Beads already copies it to both sides, and the reverb sounds as it did in version 1.
  - **Ducking** (version 2). The tail dips while the input is loud and comes back when it gets quiet.
  - **Half rate** (version 2). At the 48 kHz qualities the reverb runs at 24 kHz, between halfband filters, and keeps everything up to about 10 kHz. TONE stops brightening at about 70%.
  - **REVERB fully down** skips the reverb once its tail has faded.
- **Up to 24 grains at the top quality** (version 2), instead of 32. Beads has no room for 32 grains at 48 kHz plus this reverb: with big, dense grains the audio crackled. New grains take over the oldest one, as on stock Beads, only sooner. The other qualities keep 32.
- **A clearer delay mode** (version 2): TIME sets the delay time, DENSITY picks a pattern of taps, and the SEED light flashes on every repeat. See below.
- **Synth mode is removed.** With nothing patched into the inputs, Beads no longer switches to its wavetable synth. This frees flash and CPU time for the reverb.
- **Everything else is stock.** That covers the grains' sound, looper, CV, calibration and the saved buffer.

## Controls

The REVERB knob sets the amount, as on stock Beads.

To edit the reverb, hold **MEDIA** (the small button with the coloured light, just below DENSITY) and turn one of the large knobs:

| Knob | Setting | Range |
|---|---|---|
| SHAPE | Character | Room fully left, hall at noon, spring fully right, blending in between |
| SIZE | Size | Delay lines from 0.25× to 1.6× of the character's own |
| TIME | Decay | From about 0.3 s to over 30 s on the hall, shorter on the room |
| PITCH | Tone | Dark to bright |
| DENSITY | Diffusion | Left half: separate echoes to a smooth tail. Noon: smooth. Right half: attacks blur into a swell, and the early reflections fade out |
| REVERB (small knob) | Ducking | Off fully left. Up to 24 dB of dip on loud input |

- While you turn, the MEDIA light shows the value, going from green through yellow to red.
- When you let go, the settings are saved.
- MEDIA only changes the quality setting if you press and release it without turning anything.
- The reverb starts as a hall with about 3.3 s of decay.
- Updating from version 1 keeps your reverb settings. The diffusion value is converted so that it sounds the same, and ducking starts off.

## Delay mode

Turn SIZE fully clockwise for about a second and a half, as on stock Beads.

| Control | Without a clock | With a clock |
|---|---|---|
| TIME | Delay time, 5 ms fully left to 2.5 s near the end. The last stretch reaches the whole buffer: 4 s in stereo at the clean quality, up to 32 s in mono at the lo-fi one | Delay time as a fraction or multiple of the clock: 1/8, 1/6, 1/4, 1/3, 1/2, 2/3, 3/4, **1 at noon**, 4/3, 3/2, 2, 3, 4, 6, 8 |
| DENSITY | Pattern, in five zones from left to right: single, ping-pong, dotted (an extra tap at 3/4 of the time), triplet (1/3 and 2/3), four taps (1/4, 1/2, 3/4) | Same |
| SHAPE | Envelope of each repeat, as on stock Beads | Same |
| FEEDBACK | Number of repeats | Same |

- **Clock.** The delay follows a clock on the SEED input, or taps on the SEED button. Long-press SEED (its light glows dimly) to let it run free on TIME again.
- **Changing TIME** crossfades to the new time instead of bending the pitch.
- **Patterns.** Only the main tap feeds back. The extra taps play once per repeat, panned a little to the sides.
- **Ping-pong** works with a single input too: the first repeat comes out on the left, the next one on the right, and so on.
- **SEED light.** It flashes on every repeat of the main tap, so you can see the delay time.
- **Frozen.** The delay loops a slice of the buffer as long as the delay time. DENSITY picks which slice.
- **SIZE attenuverter.** As on stock Beads, it filters the repeats: low-pass to the left, high-pass to the right. The filter now also sits in the feedback path, so each repeat is darker, or thinner, than the last.

## Installing by audio

No programmer needed. Your calibration, settings and saved buffer are kept.

1. Download `beads_reverb_v2.wav` from the [release page](https://github.com/naturarum/eurorack/releases/tag/beads-reverb-v2).
2. Patch your computer's audio output into **IN L**, and turn **TIME** fully counterclockwise. In update mode, TIME sets the input gain.
3. Hold **FREEZE** while you power the case on. FREEZE and SEED start blinking alternately.

   Don't hold ASSIGN at the same time: FREEZE + ASSIGN at power-on erases the firmware.
4. Mute notifications and play the file. It lasts about 2 min 35 s.
   - The MEDIA light shows the incoming level. Aim for yellow or orange, and turn TIME up if it stays dim green.
   - FREEZE and SEED blink fast while data arrives, and all the LEDs light up while a block is written.
   - The module restarts by itself at the end.
5. If all the LEDs blink slowly, there was an error. Press FREEZE, adjust the level, and play the file again from the start.

**To go back to stock**, flash `beads_stock.wav` from the same release the same way. It is built from Mutable Instruments' published source. Since the bootloader is never overwritten, you can always go back.

## Building

You need the Arm GNU Toolchain 15.3. The makefile looks for it in `~/.local/opt/arm-gnu-toolchain-15.3.rel2/`; set `TOOLCHAIN_PATH` if yours lives elsewhere.

```bash
git submodule update --init stmlib stm_audio_bootloader
make -f beads/makefile wav_check
```

- `wav_check` writes `build/beads/beads.wav`. It then decodes the file with the bootloader's own demodulator and checks that the result matches the firmware byte for byte.
- The encoder, [`tools/qpsk_encoder.py`](tools/qpsk_encoder.py), is a Python 3 port of Mutable Instruments' encoder. The original needs Python 2 and numpy.
- [`test/render_reverb.sh`](test/render_reverb.sh) renders the reverb on a computer and measures decay times and levels against the stock reverb.
- Build options, set with `PROJECT_CONFIGURATION` (for example `PROJECT_CONFIGURATION=-DCPU_METER`). The makefile doesn't track them, so run `make -f beads/makefile clean` when you change them.
  - `-DCPU_METER`: test builds. The three ASSIGN LEDs show the peak load of the audio processing, from 70% (all off) to 100% (all on), 10% per LED. All three blink for a second after a block ran late, which is what a crackle is.
  - `-DBEADS_TOP_QUALITY_VOICES=n`: the number of grains at the top quality, 24 by default.
  - `-DBEADS_ITCM`: runs the audio code from the H7's ITCM. Experimental: the one build that tried it didn't start. Recovery is through the audio bootloader, as above.
  - `-DBEADS_WARM_CACHES`: leaves out the dummy grain player that stock Beads runs between audio blocks. Measurement only.
- With an ST-Link on the SWD header:
  - `make -f beads/makefile upload_app_jtag` flashes only the application.
  - `make -f beads/makefile cpu_load` prints the audio interrupt's load while the module plays.
  - Neither has been tried on hardware yet. The CPU load hasn't been measured with a debugger either.

## Credits and licence

- Beads, its bootloader and its audio update format are by Émilie Gillet, Mutable Instruments.
- The reverb draws on A. Coppola's rotation-based matrices for feedback delay networks, and on the stretched allpass spring model of V. Välimäki, J. Parker and J. Abel. References are in the source.
- The code is under the MIT license, like the rest of Mutable Instruments' STM32 projects.
- This firmware is not affiliated with or endorsed by Mutable Instruments.
