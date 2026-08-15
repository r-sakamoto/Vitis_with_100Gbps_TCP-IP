#!/bin/bash
# TOE core (10GbE, 64bit datapath) build for RTL SIMULATION with shrunk timers.
#
# Identical configuration to build_toe_10g.sh except TCP_STACK_SIM_TIMER_SCALE=1000,
# which divides all protocol timers by 1000 (RTO 1s -> ~1ms sim time) so that
# retransmission / persist / TIME_WAIT behavior can be observed in xsim.
#
# Output: buildsim/hls/toe/toe_prj/solution1/syn/verilog  (use with
#         RTL_DIR=... sim/toe_rtl/run_sim.sh)
#
# DO NOT use this build for hardware.

set -e

VIVADO_SETTINGS=/home/nvm/tools/Vivado/2022.1/settings64.sh

FPGA_PART=xcvc1902-vsvd1760-2MP-e-S
DATA_WIDTH=8        # bytes (8 = 64bit = 10GbE)
CLOCK_PERIOD=6.4    # ns (156.25 MHz)
TCP_STACK_MSS=1460
TIMER_SCALE=1000

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR=${1:-buildsim}

source "${VIVADO_SETTINGS}"

mkdir -p "${SCRIPT_DIR}/${BUILD_DIR}"
cd "${SCRIPT_DIR}/${BUILD_DIR}"

cmake .. \
    -DFPGA_PART=${FPGA_PART} \
    -DDATA_WIDTH=${DATA_WIDTH} \
    -DCLOCK_PERIOD=${CLOCK_PERIOD} \
    -DTCP_STACK_MSS=${TCP_STACK_MSS} \
    -DTCP_STACK_SIM_TIMER_SCALE=${TIMER_SCALE}

make synthesis.toe

echo ""
echo "=== TOE simulation RTL (timer scale 1/${TIMER_SCALE}) done ==="
echo "RTL: ${SCRIPT_DIR}/${BUILD_DIR}/hls/toe/toe_prj/solution1/syn/verilog"
