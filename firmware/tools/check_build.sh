#!/usr/bin/env bash
# Offline compile + link check of the firmware WITHOUT PlatformIO (used in CI / when the
# PlatformIO registry is unreachable).  Needs gcc-arm-none-eabi and the Teensy core sources:
#   git clone https://github.com/PaulStoffregen/cores  $TEENSY_SRC/cores   (+ SPI, EEPROM, WS2812Serial,
#   tonton81/FlexCAN_T4, tonton81/WDT_T4 next to it)
#   TEENSY_SRC=~/teensy-src firmware/tools/check_build.sh
set -e
T=${TEENSY_SRC:-$HOME/teensy-src}
FW=$(cd $(dirname $0)/.. && pwd)
OUT=${OUT:-/tmp/bolt-fwbuild}; mkdir -p $OUT
FLAGS="-mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb -O2 -ffunction-sections -fdata-sections -D__IMXRT1062__ -DARDUINO_TEENSY41 -DTEENSYDUINO=159 -DARDUINO=10813 -DF_CPU=600000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH"
INC="-I$T/cores/teensy4 -I$T/SPI -I$T/FlexCAN_T4 -I$T/WS2812Serial -I$T/EEPROM -I$T/WDT_T4 -I$FW/src -I$FW/lib/bolt_core -I$FW/lib/bolt_drivers -I$FW/lib/bolt_link"
CXX="arm-none-eabi-g++ $FLAGS -std=gnu++17 -fno-exceptions -fpermissive -fno-rtti -felide-constructors -Wno-error=narrowing -Wall -Wno-psabi"
$CXX $INC -c $FW/src/main.cpp -o $OUT/main.o
$CXX $INC -c $FW/lib/bolt_core/bolt_controller.cpp -o $OUT/ctrl.o
# core + libraries (for a full link)
for f in $T/cores/teensy4/*.c; do arm-none-eabi-gcc $FLAGS -std=gnu11 $INC -c $f -o $OUT/c_$(basename $f).o 2>/dev/null || echo "skip $f"; done
for f in $T/cores/teensy4/*.cpp $T/SPI/SPI.cpp $T/WS2812Serial/WS2812Serial.cpp; do $CXX $INC -c $f -o $OUT/cpp_$(basename $f).o 2>/dev/null || echo "skip $f"; done
arm-none-eabi-g++ $FLAGS -Wl,--gc-sections,--relax -T$T/cores/teensy4/imxrt1062_t41.ld $OUT/*.o -o $OUT/bolt.elf -lm -lstdc++
