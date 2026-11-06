#!/bin/sh
# Prints the CPU load of the audio interrupt of a running Beads, read over SWD
# with OpenOCD, without halting the processor. Run it through
# "make -f beads/makefile cpu_load", which sets NM and OPENOCD.
#
# The firmware keeps two counters in a struct called cpu_load: the cycles used
# by the last audio block, and the worst block since the last read. A block
# has 100000 cycles of budget (400 MHz, 4000 blocks per second).

ELF="$1"
if [ ! -f "$ELF" ]; then
  echo "No firmware at $ELF: build and flash it first."
  exit 1
fi
ADDRESS=$($NM "$ELF" | awk '$3 == "cpu_load" { print $1 }')
if [ -z "$ADDRESS" ]; then
  echo "This firmware has no cpu_load counters."
  exit 1
fi
PEAK=$(printf '0x%x' $((0x$ADDRESS + 4)))

echo "Audio interrupt load, read once a second. Ctrl-C to stop."
while true; do
  LINE=$($OPENOCD -c "init" -c "mdw 0x$ADDRESS 2" -c "mww $PEAK 0" \
      -c "shutdown" 2>&1 | grep -i "^0x$ADDRESS")
  if [ -z "$LINE" ]; then
    echo "No answer from the module: is the ST-Link connected and the case on?"
  else
    set -- $LINE
    LAST=$((0x$2))
    WORST=$((0x$3))
    awk -v last="$LAST" -v worst="$WORST" 'BEGIN {
      printf "load %5.1f %%   worst block %5.1f %%\n",
          last / 1000.0, worst / 1000.0
    }'
  fi
  sleep 1
done
