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
// Decodes a Beads firmware update .wav with the bootloader's own QPSK
// demodulator and packet decoder, following beads/bootloader/bootloader.cc,
// and compares what would be written to flash with the firmware .bin.
//
// Usage: qpsk_decode_test update.wav firmware.bin [level] [erase_stall_s]
//            [write_stall_s] [noise]
//   level          peak of the signal reaching the demodulator, relative to
//                  the bootloader's 16-bit full scale (both channels summed).
//   erase_stall_s  seconds of audio lost while a 128 KB sector is erased and
//                  its first 64 KB written.
//   write_stall_s  seconds lost while another 64 KB block is written.
//   noise          white noise RMS, relative to full scale.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "stm_audio_bootloader/qpsk/demodulator.h"
#include "stm_audio_bootloader/qpsk/packet_decoder.h"

using namespace stm_audio_bootloader;

const double kSampleRate = 48000.0;
const double kModulationRate = 6000.0;
const double kBitRate = 12000.0;
const uint32_t kStartAddress = 0x08020000;
const uint32_t kBlockSize = 0x10000;
const uint16_t kPacketsPerBlock = kBlockSize / kPacketSize;
const size_t kCodecBlockSize = 12;

bool ReadFile(const char* name, std::vector<uint8_t>* data) {
  FILE* f = fopen(name, "rb");
  if (!f) return false;
  uint8_t buffer[4096];
  size_t n;
  while ((n = fread(buffer, 1, sizeof(buffer), f)) > 0) {
    data->insert(data->end(), buffer, buffer + n);
  }
  fclose(f);
  return true;
}

bool ReadWav(const char* name, std::vector<int16_t>* samples) {
  std::vector<uint8_t> data;
  if (!ReadFile(name, &data) || data.size() < 12 ||
      memcmp(&data[0], "RIFF", 4) || memcmp(&data[8], "WAVE", 4)) {
    return false;
  }
  size_t p = 12;
  uint16_t channels = 0, bits = 0;
  uint32_t rate = 0;
  while (p + 8 <= data.size()) {
    uint32_t size = data[p + 4] | data[p + 5] << 8 | data[p + 6] << 16 |
        data[p + 7] << 24;
    const uint8_t* body = &data[p + 8];
    if (!memcmp(&data[p], "fmt ", 4)) {
      channels = body[2] | body[3] << 8;
      rate = body[4] | body[5] << 8 | body[6] << 16 | body[7] << 24;
      bits = body[14] | body[15] << 8;
    } else if (!memcmp(&data[p], "data", 4)) {
      if (channels != 1 || bits != 16 || rate != 48000) {
        fprintf(stderr, "expected 48 kHz 16-bit mono\n");
        return false;
      }
      size = std::min<size_t>(size, data.size() - p - 8);
      for (size_t i = 0; i + 1 < size; i += 2) {
        samples->push_back(int16_t(body[i] | body[i + 1] << 8));
      }
      return true;
    }
    p += 8 + size + (size & 1);
  }
  return false;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s update.wav firmware.bin [level] "
            "[erase_stall_s] [write_stall_s] [noise]\n", argv[0]);
    return 2;
  }
  std::vector<int16_t> wav;
  std::vector<uint8_t> firmware;
  if (!ReadWav(argv[1], &wav)) {
    fprintf(stderr, "cannot read %s\n", argv[1]);
    return 2;
  }
  if (!ReadFile(argv[2], &firmware)) {
    fprintf(stderr, "cannot read %s\n", argv[2]);
    return 2;
  }
  const double level = argc > 3 ? atof(argv[3]) : 0.4;
  const double erase_stall = argc > 4 ? atof(argv[4]) : 0.0;
  const double write_stall = argc > 5 ? atof(argv[5]) : 0.0;
  const double noise = argc > 6 ? atof(argv[6]) : 0.0;

  static Demodulator demodulator;
  static PacketDecoder decoder;
  decoder.Init(20000, true);
  demodulator.Init(
      kModulationRate / kSampleRate * 4294967296.0,
      kSampleRate / kModulationRate,
      2.0 * kSampleRate / kBitRate);
  demodulator.SyncCarrier(true);
  decoder.Reset();

  std::vector<uint8_t> flash;  // Contents written from kStartAddress on.
  static uint8_t rx_buffer[kBlockSize];
  uint32_t current_address = kStartAddress;
  uint16_t packet_index = 0;
  size_t packets = 0;
  srand(1);

  size_t position = 0;
  bool done = false;
  bool error = false;
  while (!done && !error && position < wav.size()) {
    // Codec interrupt: one block of samples into the demodulator.
    for (size_t i = 0; i < kCodecBlockSize && position < wav.size(); ++i) {
      double x = wav[position++] / 32767.0 * level;
      if (noise > 0.0) {
        double u = 0.0;
        for (int k = 0; k < 12; ++k) u += rand() / double(RAND_MAX);
        x += (u - 6.0) * noise;
      }
      int32_t sample = int32_t(x * 32767.0);
      if (sample < -32768) sample = -32768;
      if (sample > 32767) sample = 32767;
      demodulator.PushSample(2048 + (sample >> 4));
    }

    // Main loop.
    if (demodulator.state() == DEMODULATOR_STATE_OVERFLOW) {
      fprintf(stderr, "demodulator overflow\n");
      error = true;
      break;
    }
    demodulator.ProcessAtLeast(32);
    while (demodulator.available() && !error && !done) {
      uint8_t symbol = demodulator.NextSymbol();
      PacketDecoderState state = decoder.ProcessSymbol(symbol);
      switch (state) {
        case PACKET_DECODER_STATE_OK:
          {
            const uint8_t* source = decoder.packet_data();
            memcpy(rx_buffer + (packet_index % kPacketsPerBlock) * kPacketSize,
                   source, kPacketSize);
            ++packet_index;
            ++packets;
            if ((packet_index % kPacketsPerBlock) == 0) {
              bool erase = (current_address % 0x20000) == 0;
              printf("  block 0x%08x written at %6.2f s%s\n",
                     current_address, position / kSampleRate,
                     erase ? " (sector erased)" : "");
              flash.insert(flash.end(), rx_buffer, rx_buffer + kBlockSize);
              current_address += kBlockSize;
              // Audio that arrives while the flash is busy is lost.
              position += size_t(
                  (erase ? erase_stall : write_stall) * kSampleRate);
              decoder.Reset();
              demodulator.SyncCarrier(false);
            } else {
              decoder.Reset();
              demodulator.SyncDecision();
            }
          }
          break;
        case PACKET_DECODER_STATE_ERROR_SYNC:
        case PACKET_DECODER_STATE_ERROR_CRC:
          fprintf(stderr, "%s error at %.2f s, after %zu packets\n",
                  state == PACKET_DECODER_STATE_ERROR_CRC ? "CRC" : "sync",
                  position / kSampleRate, packets);
          error = true;
          break;
        case PACKET_DECODER_STATE_END_OF_TRANSMISSION:
          done = true;
          break;
        default:
          break;
      }
    }
  }

  if (error) {
    printf("FAIL: decoding error\n");
    return 1;
  }
  if (!done) {
    printf("FAIL: no end of transmission (%zu packets)\n", packets);
    return 1;
  }
  std::vector<uint8_t> expected(firmware);
  expected.resize((expected.size() + kBlockSize - 1) / kBlockSize * kBlockSize,
                  0xff);
  if (flash != expected) {
    size_t i = 0;
    while (i < flash.size() && i < expected.size() && flash[i] == expected[i]) {
      ++i;
    }
    printf("FAIL: flash differs from firmware at offset 0x%zx "
           "(%zu bytes written, %zu expected)\n",
           i, flash.size(), expected.size());
    return 1;
  }
  printf("OK: %zu packets, %zu bytes match the firmware, "
         "end of transmission at %.2f s\n",
         packets, flash.size(), position / kSampleRate);
  return 0;
}
