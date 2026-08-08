#!/bin/bash
# TOE core (10GbE, 64bit datapath) build & IP export script
#
# Usage: ./build_toe_10g.sh [build_dir]
#   build_dir: default "build10g"
#
# Output:
#   <build_dir>/hls/toe/toe_prj/solution1/impl/export.zip (ipname: toe_10g)
#   iprepo10g/toe_10g/  (IP repository for Vivado; VLNV ethz.systems:hls:toe_10g:1.7,
#                        distinct from the 512bit ethz.systems:hls:toe:1.7)

set -e

VIVADO_SETTINGS=/home/nvm/tools/Vivado/2022.1/settings64.sh

FPGA_PART=xcvc1902-vsvd1760-2MP-e-S
DATA_WIDTH=8        # bytes (8 = 64bit = 10GbE)
CLOCK_PERIOD=6.4    # ns (156.25 MHz)
TCP_STACK_MSS=1460

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR=${1:-build10g}

source "${VIVADO_SETTINGS}"

mkdir -p "${SCRIPT_DIR}/${BUILD_DIR}"
cd "${SCRIPT_DIR}/${BUILD_DIR}"

cmake .. \
    -DFPGA_PART=${FPGA_PART} \
    -DDATA_WIDTH=${DATA_WIDTH} \
    -DCLOCK_PERIOD=${CLOCK_PERIOD} \
    -DTCP_STACK_MSS=${TCP_STACK_MSS}

make synthesis.toe

# Export as "toe_10g" so the VLNV does not clash with the 512bit toe IP
cd "${SCRIPT_DIR}/${BUILD_DIR}/hls/toe"
cat > export_toe_10g.tcl <<'EOF'
open_project toe_prj
open_solution solution1
export_design -format ip_catalog -ipname "toe_10g" -display_name "10G TCP Offload Engine (64bit)" -description "TCP Offload Engine supporting 10Gbps line rate, 64bit datapath." -vendor "ethz.systems" -version "1.7"
exit
EOF
vitis_hls -f export_toe_10g.tcl

EXPORT_ZIP="${SCRIPT_DIR}/${BUILD_DIR}/hls/toe/toe_prj/solution1/impl/export.zip"
if [ ! -f "${EXPORT_ZIP}" ]; then
    echo "ERROR: export.zip not found: ${EXPORT_ZIP}" >&2
    exit 1
fi

# Install into the 10G IP repository referenced by the Vivado project
IPREPO_DIR="${SCRIPT_DIR}/iprepo10g"
mkdir -p "${IPREPO_DIR}"
rm -rf "${IPREPO_DIR}/toe_10g"
cp -r "${SCRIPT_DIR}/${BUILD_DIR}/hls/toe/toe_prj/solution1/impl/ip" "${IPREPO_DIR}/toe_10g"

echo ""
echo "=== TOE (toe_10g) IP export done ==="
ls -la "${EXPORT_ZIP}"
echo "installed: ${IPREPO_DIR}/toe_10g"
