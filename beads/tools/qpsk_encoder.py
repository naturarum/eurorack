#!/usr/bin/env python3
#
# Copyright 2013 Emilie Gillet.
#
# Author: Emilie Gillet (emilie.o.gillet@gmail.com)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# See http://creativecommons.org/licenses/MIT/ for more information.
#
# -----------------------------------------------------------------------------
#
# QPSK encoder for converting firmware .bin files into .wav files for the
# Mutable Instruments audio bootloader.
#
# Python 3 port of stm_audio_bootloader/qpsk/encoder.py, as of upstream
# commit b376bd6, which adds the STM32H7 target used by Beads. The upstream
# script needs Python 2 and numpy; this one needs neither. The signal is the
# same: integer divisions are kept as integer divisions, and the scrambler,
# packet layout, CRC, pauses and phase-continuous carrier are unchanged.
# Port by naturarum, 2026.

import argparse
import math
import sys
import wave
from array import array
import zlib


class Scrambler(object):

  def __init__(self, seed=0):
    self._state = seed

  def scramble(self, data):
    out = bytearray(len(data))
    state = self._state
    for index, byte in enumerate(data):
      out[index] = byte ^ (state >> 24)
      state = (state * 1664525 + 1013904223) & 0xffffffff
    self._state = state
    return bytes(out)


class QpskEncoder(object):

  def __init__(
      self,
      sample_rate=48000,
      carrier_frequency=2000,
      bit_rate=8000,
      packet_size=256,
      scramble=False):
    period = sample_rate // carrier_frequency
    symbol_time = sample_rate // (bit_rate // 2)

    assert (symbol_time % period == 0) or (period % symbol_time == 0)
    assert (sample_rate % carrier_frequency) == 0
    assert (sample_rate % bit_rate) == 0

    self._sr = sample_rate
    self._br = bit_rate
    self._carrier_frequency = carrier_frequency
    self._sample_index = 0
    self._packet_size = packet_size
    self._scrambler = Scrambler(0) if scramble else None

  def _encode(self, symbol_stream):
    # Each symbol holds two bits: bit 0 drives the sine (Q) component, bit 1
    # the cosine (I) component, each at +/- 1/sqrt(2). A symbol lasts
    # 2 * sample_rate / bit_rate samples. The carrier phase runs on from one
    # call to the next.
    ratio = self._sr // self._br * 2
    omega = 2 * math.pi * self._carrier_frequency
    sr = float(self._sr)
    root_two = math.sqrt(2.0)
    levels = [(2 * (s % 2) - 1) / root_two for s in range(4)], \
             [(2 * (s // 2) - 1) / root_two for s in range(4)]
    out = array('d')
    n = self._sample_index
    sin = math.sin
    cos = math.cos
    for symbol in symbol_stream:
      q = levels[0][symbol]
      i = levels[1][symbol]
      for _ in range(ratio):
        phase = omega * (n / sr)
        out.append(q * sin(phase) + i * cos(phase))
        n += 1
    self._sample_index = n
    return out

  def _code_blank(self, duration):
    num_zeros = int(duration * self._br / 8) * 4
    return self._encode([0] * num_zeros)

  def _code_packet(self, data):
    assert len(data) <= self._packet_size
    if len(data) != self._packet_size:
      data = data + b'\xff' * (self._packet_size - len(data))

    if self._scrambler:
      data = self._scrambler.scramble(data)

    crc = zlib.crc32(data) & 0xffffffff

    # 8x 0 for the PLL ; 4x 0x99 for the edge detector ; 4x 0xcc for syncing.
    preamble = [0] * 8 + [0x99] * 4 + [0xcc] * 4
    crc_bytes = [crc >> 24, (crc >> 16) & 0xff, (crc >> 8) & 0xff, crc & 0xff]
    stream = preamble + list(data) + crc_bytes

    symbol_stream = []
    for byte in stream:
      symbol_stream.append((byte >> 6) & 0x3)
      symbol_stream.append((byte >> 4) & 0x3)
      symbol_stream.append((byte >> 2) & 0x3)
      symbol_stream.append((byte >> 0) & 0x3)

    return self._encode(symbol_stream)

  def code_intro(self):
    # One second of silence (the carrier phase does not advance), then one
    # second of blank carrier.
    yield array('d', bytes(8 * self._sr))
    yield self._code_blank(1.0)

  def code_outro(self, duration=1.0):
    yield self._code_blank(duration)

  def code(self, data, page_size=1024, blank_duration=0.06):
    if len(data) % page_size != 0:
      tail = page_size - (len(data) % page_size)
      data = data + b'\xff' * tail

    offset = 0
    remaining_bytes = len(data)
    num_packets_written = 0
    while remaining_bytes:
      size = min(remaining_bytes, self._packet_size)
      yield self._code_packet(data[offset:offset + size])
      num_packets_written += 1
      if num_packets_written == page_size // self._packet_size:
        yield self._code_blank(blank_duration)
        num_packets_written = 0
      remaining_bytes -= size
      offset += size


STM32F4_SECTOR_BASE_ADDRESS = [
  0x08000000,
  0x08004000,
  0x08008000,
  0x0800C000,
  0x08010000,
  0x08020000,
  0x08040000,
  0x08060000,
  0x08080000,
  0x080A0000,
  0x080C0000,
  0x080E0000
]

STM32H7_SECTOR_BASE_ADDRESS = [0x08000000 + 0x20000 * i for i in range(16)]

PAGE_SIZE = {'stm32f1': 1024, 'stm32f3': 2048}
PAUSE = {'stm32f1': 0.06, 'stm32f3': 0.15}

STM32F4_BLOCK_SIZE = 16384
STM32F4_APPLICATION_START = 0x08008000

STM32H7_BLOCK_SIZE = 0x10000
STM32H7_APPLICATION_START = 0x08020000


class WavWriter(object):
  """16-bit PCM writer. Floats are clipped to [-1, 1], scaled by 32767 and
  truncated towards zero, like the numpy cast in audio_stream_writer.py."""

  def __init__(self, file_name, sample_rate):
    self._wav = wave.open(file_name, 'wb')
    self._wav.setnchannels(1)
    self._wav.setsampwidth(2)
    self._wav.setframerate(sample_rate)
    self.num_samples = 0

  def append(self, signal):
    pcm = array('h', (
        int(max(-1.0, min(1.0, x)) * 32767) for x in signal))
    if sys.byteorder != 'little':
      pcm.byteswap()
    self._wav.writeframes(pcm.tobytes())
    self.num_samples += len(pcm)

  def close(self):
    self._wav.close()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('-k', '--scramble', action='store_true', default=False,
                      help='Randomize data stream')
  parser.add_argument('-s', '--sample_rate', type=int, default=48000,
                      help='Sample rate in Hz')
  parser.add_argument('-c', '--carrier_frequency', type=int, default=6000,
                      help='Carrier frequency in Hz')
  parser.add_argument('-b', '--baud_rate', type=int, default=12000,
                      help='Baudrate in bps')
  parser.add_argument('-p', '--packet_size', type=int, default=256,
                      help='Packet size in bytes')
  parser.add_argument('-o', '--output_file', default=None, metavar='FILE',
                      help='Write output file to FILE')
  parser.add_argument('-t', '--target', default='stm32f1', metavar='TARGET',
                      help='Set page size and erase time for TARGET')
  parser.add_argument('firmware', help='Firmware .bin file')
  options = parser.parse_args()

  if options.target not in ['stm32f1', 'stm32f3', 'stm32f4', 'stm32h7']:
    sys.exit('Unknown target: %s' % options.target)

  with open(options.firmware, 'rb') as f:
    data = f.read()

  output_file = options.output_file
  if not output_file:
    if '.bin' in options.firmware:
      output_file = options.firmware.replace('.bin', '.wav')
    else:
      output_file = options.firmware + '.wav'

  encoder = QpskEncoder(
      options.sample_rate,
      options.carrier_frequency,
      options.baud_rate,
      options.packet_size,
      options.scramble)
  writer = WavWriter(output_file, options.sample_rate)

  for block in encoder.code_intro():
    writer.append(block)

  blank_duration = 1.0
  if options.target in PAGE_SIZE:
    for block in encoder.code(
        data, PAGE_SIZE[options.target], PAUSE[options.target]):
      if len(block):
        writer.append(block)
  else:
    if options.target == 'stm32f4':
      block_size = STM32F4_BLOCK_SIZE
      start_address = STM32F4_APPLICATION_START
      sector_base = STM32F4_SECTOR_BASE_ADDRESS
      erase_pause = 3.5
    else:
      block_size = STM32H7_BLOCK_SIZE
      start_address = STM32H7_APPLICATION_START
      sector_base = STM32H7_SECTOR_BASE_ADDRESS
      erase_pause = 3.0
    for x in range(0, len(data), block_size):
      address = start_address + x
      chunk = data[x:x + block_size]
      pause = erase_pause if address in sector_base else 0.2
      for block in encoder.code(chunk, block_size, pause):
        if len(block):
          writer.append(block)
    blank_duration = 5.0

  for block in encoder.code_outro(blank_duration):
    writer.append(block)
  writer.close()

  duration = writer.num_samples / float(options.sample_rate)
  print('%s: %d bytes -> %s (%d:%02d)' % (
      options.firmware, len(data), output_file,
      int(duration) // 60, int(duration) % 60))


if __name__ == '__main__':
  main()
