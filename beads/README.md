# Beads with a morphing reverb

Unofficial firmware for Mutable Instruments Beads. The granular processor is unchanged. The built-in plate reverb is replaced by one whose character moves continuously from room through hall to spring, and all its settings can be changed from the panel.

So far it has been tested on one module.

## What changed

- **A new reverb**, in [`dsp/fx/morph_reverb.h`](dsp/fx/morph_reverb.h):
  - An 8-line feedback delay network, mixed by a matrix built from rotations. Diffusion sets the rotation angles.
  - Four of the lines run through chains of stretched allpass filters, which give the spring its chirp.
  - The spring fades in as you turn towards it; there is no switch.
  - Early reflections and four input diffusers sit in front of the network.
- **Synth mode is removed.** With nothing patched into the inputs, Beads no longer switches to its wavetable synth. This frees flash and CPU time for the reverb.
- **Everything else is stock.** That covers grains, delay, looper, CV, calibration and the saved buffer.

## Controls

The REVERB knob sets the amount, as on stock Beads.

To edit the reverb, hold **MEDIA** (the small button with the coloured light, just below DENSITY) and turn one of the large knobs:

| Knob | Setting | Range |
|---|---|---|
| SHAPE | Character | Room fully left, hall at noon, spring fully right, blending in between |
| SIZE | Size | Delay lines from 0.25× to 1.6× of the character's own |
| TIME | Decay | From about 0.3 s to over 30 s on the hall, shorter on the room |
| PITCH | Tone | Dark to bright |
| DENSITY | Diffusion | Separate echoes to a smooth wash |

- While you turn, the MEDIA light shows the value, going from green through yellow to red.
- When you let go, the settings are saved.
- MEDIA only changes the quality setting if you press and release it without turning anything.
- The reverb starts as a hall with about 3.3 s of decay.

## Installing by audio

No programmer needed. Your calibration, settings and saved buffer are kept.

1. Download `beads_reverb_v1.wav` from the [release page](https://github.com/naturarum/eurorack/releases/tag/beads-reverb-v1).
2. Patch your computer's audio output into **IN L**, and turn **TIME** fully counterclockwise. In update mode, TIME sets the input gain.
3. Hold **FREEZE** while you power the case on. FREEZE and SEED start blinking alternately.

   Don't hold ASSIGN at the same time: FREEZE + ASSIGN at power-on erases the firmware.
4. Mute notifications and play the file. It lasts 2 min 34 s.
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
- With an ST-Link on the SWD header:
  - `make -f beads/makefile upload_app_jtag` flashes only the application.
  - `make -f beads/makefile cpu_load` prints the audio interrupt's load while the module plays.
  - Neither has been tried on hardware yet. The CPU load hasn't been measured with a debugger either.

## Credits and licence

- Beads, its bootloader and its audio update format are by Émilie Gillet, Mutable Instruments.
- The reverb draws on A. Coppola's rotation-based matrices for feedback delay networks, and on the stretched allpass spring model of V. Välimäki, J. Parker and J. Abel. References are in the source.
- The code is under the MIT license, like the rest of Mutable Instruments' STM32 projects.
- This firmware is not affiliated with or endorsed by Mutable Instruments.
