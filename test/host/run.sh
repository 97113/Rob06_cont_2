#!/bin/sh
# ---------------------------------------------------------------------------
# Host tests. No hardware, no ESP-IDF, no PlatformIO - just a C++17 compiler.
#
#     test/host/run.sh
#
# The parts of the firmware that are pure logic (the CAN transport, the
# controller's safety layer, the trajectory generator, the fixed point
# scaling) are compiled against small stubs in test/host/stubs/ and driven by
# a scriptable fake motor on a virtual bus. Time is virtual, so the whole
# suite runs in well under a second and gives the same answer every time.
#
# This does NOT replace building the firmware - it cannot catch anything that
# depends on M5Unified, the real TWAI driver or the SD card. Run `pio run` for
# that.
# ---------------------------------------------------------------------------
set -e

cd "$(dirname "$0")/../.."
OUT="${TMPDIR:-/tmp}/rs06-host-tests"
mkdir -p "$OUT"

CXX="${CXX:-g++}"
FLAGS="-std=gnu++17 -Wall -Wextra -Werror -O1 -I test/host/stubs -I src"

echo "building..."
$CXX $FLAGS \
  test/host/test_rs06_driver.cpp test/host/fake_bus.cpp \
  src/rs06_driver.cpp \
  -o "$OUT/test_rs06_driver"

$CXX $FLAGS -DCTRL_HOST_TEST \
  test/host/test_controller.cpp test/host/fake_bus.cpp test/host/stubs_impl.cpp \
  src/controller.cpp src/rs06_driver.cpp src/calib.cpp \
  -o "$OUT/test_controller"

# Compile every firmware translation unit, including the ones no test drives
# yet (UI, serial link, logger, CAN scanner). They only need M5Unified, SD and
# SPI to exist, not to work, and this is what turns "never compiled" into a
# one second check. -Werror, so warnings are not allowed to accumulate.
echo "compiling all firmware sources..."
for f in src/*.cpp; do
  $CXX $FLAGS -c "$f" -o "$OUT/$(basename "$f" .cpp).o"
done

fail=0
"$OUT/test_rs06_driver" || fail=1
"$OUT/test_controller"  || fail=1

# The Core2 and the PC console agree on the wire format only by hand. This
# reads both sources and compares the shapes, so a column added on one side
# and forgotten on the other fails here instead of silently plotting nothing.
echo
python3 test/host/check_protocol_contract.py || fail=1

echo
if [ $fail -eq 0 ]; then
  echo "ALL HOST TESTS PASSED"
else
  echo "HOST TESTS FAILED"
fi
exit $fail
