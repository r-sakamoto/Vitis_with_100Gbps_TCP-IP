#!/bin/bash
# TOE core (10GbE, 64bit datapath) build with RX_DDR_BYPASS=0
# (RX payload goes through DDR via datamover cmd/data/status instead of the
#  bypass FIFO; enables the single-gap out-of-order receive path).
#
# Usage: ./build_toe_10g_nobypass.sh [build_dir] [timer_scale]
#   build_dir   : default "build10g_nb"
#   timer_scale : default 1 (hardware timing). Use 1000 for RTO simulation.
#
# Output: <build_dir>/hls/toe/toe_prj/solution1/syn/verilog

set -e

VIVADO_SETTINGS=/home/nvm/tools/Vivado/2022.1/settings64.sh

FPGA_PART=xcvc1902-vsvd1760-2MP-e-S
DATA_WIDTH=8        # bytes (8 = 64bit = 10GbE)
CLOCK_PERIOD=6.4    # ns (156.25 MHz)
TCP_STACK_MSS=1460

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR=${1:-build10g_nb}
TIMER_SCALE=${2:-1}

source "${VIVADO_SETTINGS}"

mkdir -p "${SCRIPT_DIR}/${BUILD_DIR}"
cd "${SCRIPT_DIR}/${BUILD_DIR}"

cmake .. \
    -DFPGA_PART=${FPGA_PART} \
    -DDATA_WIDTH=${DATA_WIDTH} \
    -DCLOCK_PERIOD=${CLOCK_PERIOD} \
    -DTCP_STACK_MSS=${TCP_STACK_MSS} \
    -DTCP_STACK_RX_DDR_BYPASS_EN=0 \
    -DTCP_STACK_SIM_TIMER_SCALE=${TIMER_SCALE}

make synthesis.toe

echo ""
echo "=== TOE RX_DDR_BYPASS=0 RTL (timer scale 1/${TIMER_SCALE}) done ==="
echo "RTL: ${SCRIPT_DIR}/${BUILD_DIR}/hls/toe/toe_prj/solution1/syn/verilog"
