#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
g++ -std=c++17 -Wall -Wextra -Werror -Wno-unused-parameter \
  -I. -Itests/host/include -include tests/host/fake_esphome.h \
  tests/host/bridge_test.cpp components/jsdrive2/jsdrive2.cpp \
  -o /tmp/jsdrive2-bridge-test
/tmp/jsdrive2-bridge-test
