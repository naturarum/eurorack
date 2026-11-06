# Copyright 2019 Emilie Gillet.
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

# System specifications
F_CRYSTAL      = 8000000L
F_CPU          = 400000000L
SYSCLOCK       = SYSCLK_FREQ_400MHz
FAMILY         = h7xx
# USB            = enabled

APPLICATION_LARGE = TRUE
BOOTLOADER        = beads_bootloader

# Prefered upload command
# UPLOAD_COMMAND  = upload_combo_jtag_erase_first_sector_and_settings
UPLOAD_COMMAND  = upload_combo_jtag_erase_first_sector

# Packages to build
TARGET         = beads
PACKAGES       = beads \
		beads/drivers \
		beads/dsp \
		stmlib/dsp \
		stmlib/utils \
		stmlib/ui \
		stmlib/system
RESOURCES      = beads/resources

# Also tested with latest gcc version (8.3.1). Improvement of code size, no
# improvement of CPU use, much longer compile time.
# Default setup: Arm GNU Toolchain 15.3 unpacked in ~/.local/opt, and OpenOCD
# 0.12 from Homebrew on Apple silicon. Override on the command line or in the
# environment if yours live elsewhere.
TOOLCHAIN_PATH ?= $(HOME)/.local/opt/arm-gnu-toolchain-15.3.rel2/
OPENOCD_BIN_PATH ?= /opt/homebrew/bin/
OPENOCD_SCRIPT_PATH ?= /opt/homebrew/share/openocd/scripts/
GOLDEN_FIRMWARE = beads/hardware_design/golden_firmware/beads_golden_bootloader_combo.bin

include stmlib/makefile.inc

# Modern GCC (15.x) defaults to C++17, which rejects the 'register' keyword
# used by the ST HAL headers. Pin the dialect upstream was built with.
CPPFLAGS += -std=gnu++14

# Rule for building the firmware update file. The encoder in the
# stm_audio_bootloader submodule needs Python 2 and has no stm32h7 target at
# the pinned commit; beads/tools/qpsk_encoder.py is a Python 3 port of the
# upstream version that has it.
wav:  $(TARGET_BIN)
	python3 beads/tools/qpsk_encoder.py \
		-t stm32h7 -k -s 48000 -b 12000 -c 6000 -p 256 \
		$(TARGET_BIN)

# Decodes the update file with the bootloader's own demodulator and packet
# decoder, with audio lost during flash writes and some noise, and checks that
# what would be written matches the firmware.
wav_check: wav
	c++ -std=c++14 -O2 -I. -o $(BUILD_DIR)qpsk_decode_test \
		beads/test/qpsk_decode_test.cc \
		stm_audio_bootloader/qpsk/demodulator.cc \
		stm_audio_bootloader/qpsk/packet_decoder.cc
	$(BUILD_DIR)qpsk_decode_test $(TARGET_BIN:.bin=.wav) $(TARGET_BIN) \
		0.3 2.5 0.15 0.01

# Flashes the application only, at 0x08020000: the bootloader, the saved
# buffer and the settings are left untouched. Takes a few seconds.
upload_app_jtag: $(TARGET_BIN)
	$(BASE_JTAG_CMD) \
		-c "flash write_image erase $(TARGET_BIN) 0x08020000" \
		-c "verify_image $(TARGET_BIN) 0x08020000" \
		$(POSTLUDE)

# Prints the CPU load of the audio interrupt once a second, read over SWD
# while the module keeps running. Uses the ELF of the last build, which must
# be the firmware on the module. Ctrl-C to stop.
cpu_load:
	NM="$(NM)" \
	OPENOCD="$(OPENOCD_BIN_PATH)openocd -s $(OPENOCD_SCRIPT_PATH) -f $(OPENOCD_INTERFACE_SCRIPT) -f $(OPENOCD_TARGET_SCRIPT)" \
	sh beads/tools/cpu_load.sh $(TARGET_ELF)

# Rule for dumping the settings block
image_settings:
	$(BASE_JTAG_CMD) \
		-c "dump_image beads_settings.bin 0x081e0000 0x020000" \
		$(POSTLUDE)

# Rule for dumping the settings block
erase_settings:
	$(BASE_JTAG_CMD) \
		-c "flash erase_address 0x081e0000 0x20000" \
		$(POSTLUDE)

write_settings: $(COMBO_BIN)
	$(BASE_JTAG_CMD) \
				-c "flash write_image erase beads_settings_ok.bin 0x081e0000" \
				-c "verify_image beads_settings_ok.bin 0x081e0000" \
				$(POSTLUDE)

# Rule for dumping the buffer save block
buffer_image_jtag:
	$(BASE_JTAG_CMD) \
		-c "dump_image beads_buffer.bin 0x08100000 0x0c0000" \
		$(POSTLUDE)

# Rule for dumping a whole image
image_all:
	$(BASE_JTAG_CMD) \
		-c "dump_image beads_full.bin 0x08000000 0x200000" \
		$(POSTLUDE)

# Writes back a whole image dumped with image_all: bootloader, firmware,
# saved buffer, settings and calibration. Calibration differs from unit to
# unit: restore each module from its own dump, with BEADS_IMAGE=<file>.
BEADS_IMAGE ?= beads_full.bin
restore_image_all:
	$(BASE_JTAG_CMD) \
		-c "flash write_image erase $(BEADS_IMAGE) 0x08000000" \
		-c "verify_image $(BEADS_IMAGE) 0x08000000" \
		$(POSTLUDE)

upload_combo_jtag_erase_first_sector_and_settings: $(COMBO_BIN)
	$(BASE_JTAG_CMD) \
				-c "flash erase_address 0x08000000 0x20000" \
				-c "flash erase_address 0x081e0000 0x20000" \
				-c "reset halt" \
				-c "flash write_image erase $(COMBO_BIN) 0x08000000" \
				-c "verify_image $(COMBO_BIN) 0x08000000" \
				$(POSTLUDE)

# Factory write
factory_write:
	$(BASE_JTAG_CMD) \
				-c "flash erase_address 0x08000000 0x20000" \
				-c "flash erase_address 0x081e0000 0x20000" \
				-c "reset halt" \
				-c "flash write_image erase $(GOLDEN_FIRMWARE) 0x08000000" \
				-c "verify_image $(GOLDEN_FIRMWARE) 0x08000000" \
				$(POSTLUDE)

factory_write_keep_calibration:
	$(BASE_JTAG_CMD) \
				-c "flash erase_address 0x08000000 0x20000" \
				-c "reset halt" \
				-c "flash write_image erase $(GOLDEN_FIRMWARE) 0x08000000" \
				-c "verify_image $(GOLDEN_FIRMWARE) 0x08000000" \
				$(POSTLUDE)

# Factory writes for older golden images
factory_write_batch_1:
	$(BASE_JTAG_CMD) \
				-c "flash erase_address 0x08000000 0x20000" \
				-c "flash erase_address 0x081e0000 0x20000" \
				-c "reset halt" \
				-c "flash write_image erase beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_1.bin 0x08000000" \
				-c "verify_image beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_1.bin 0x08000000" \
				$(POSTLUDE)

factory_write_batch_2_aborted:
	$(BASE_JTAG_CMD) \
				-c "flash erase_address 0x08000000 0x20000" \
				-c "flash erase_address 0x081e0000 0x20000" \
				-c "reset halt" \
				-c "flash write_image erase beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_2_aborted.bin 0x08000000" \
				-c "verify_image beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_2_aborted.bin 0x08000000" \
				$(POSTLUDE)

factory_write_batch_2_candidate:
	$(BASE_JTAG_CMD) \
				-c "flash erase_address 0x08000000 0x20000" \
				-c "flash erase_address 0x081e0000 0x20000" \
				-c "reset halt" \
				-c "flash write_image erase beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_2_candidate.bin 0x08000000" \
				-c "verify_image beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_2_candidate.bin 0x08000000" \
				$(POSTLUDE)

factory_write_batch_2_candidate_safe:
	$(BASE_JTAG_CMD) \
				-c "flash erase_address 0x08000000 0x20000" \
				-c "flash erase_address 0x081e0000 0x20000" \
				-c "reset halt" \
				-c "flash write_image erase beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_2_candidate_safe.bin 0x08000000" \
				-c "verify_image beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_2_candidate_safe.bin 0x08000000" \
				$(POSTLUDE)

factory_write_batch_2_candidate_eraser:
	$(BASE_JTAG_CMD) \
				-c "flash erase_address 0x08000000 0x20000" \
				-c "flash erase_address 0x081e0000 0x20000" \
				-c "reset halt" \
				-c "flash write_image erase beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_2_candidate_eraser.bin 0x08000000" \
				-c "verify_image beads/hardware_design/golden_firmware/beads_golden_bootloader_combo_batch_2_candidate_eraser.bin 0x08000000" \
				$(POSTLUDE)

