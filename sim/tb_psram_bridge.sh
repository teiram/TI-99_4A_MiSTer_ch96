#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p sim/vvp sim/vcd
iverilog -g2012 -o sim/vvp/tb_psram_bridge.vvp \
  rtl/psram.sv rtl/psram_cal3.sv rtl/psram_dps.sv rtl/psram_bridge.sv \
  sim/aps6404l_model.sv sim/tb_psram_bridge.sv
vvp sim/vvp/tb_psram_bridge.vvp
