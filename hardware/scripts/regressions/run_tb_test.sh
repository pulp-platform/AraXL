#!/usr/bin/env bash
# Copyright 2026 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)

# Register each test once. Both simulator backends and the suite use this list.
# Arguments: name, package group, RTL sources (space separated), completion
# printf format (one %s for the cluster count), extra completion marker, and
# optional Verilator warning flags. Top-level names follow <name>_tb.
tests=()
declare -A package_groups rtl_sources pass_formats extra_markers verilator_flags
register_test() {
  tests+=("$1")
  package_groups[$1]=$2
  rtl_sources[$1]=$3
  pass_formats[$1]=$4
  extra_markers[$1]=${5:-}
  verilator_flags[$1]=${6:-}
}

register_test align_transition ara hardware/src/vlsu/align_stage.sv \
  'PASS align transition: C=%s stall=0' 'PASS align transition: C=2 stall=1'
register_test axi_inval_filter axi hardware/src/axi_inval_filter.sv \
  'PASS AXI invalidation: C=%s '
register_test indexed_read ara hardware/src/vlsu/global_ldst.sv \
  'PASS indexed read backpressure: C=%s '
register_test mask_transfer ara hardware/src/vlsu/shuffle_stage.sv \
  'PASS mask transfers: C=%s,'
register_test reshuffle_dispatcher ara hardware/src/ara_dispatcher.sv \
  'RESHUFFLE DISPATCHER PASS C=%s' 'RESHUFFLE DISPATCHER REGRESSION PASS (' '-Wno-ENUMVALUE'
register_test store_transition ara hardware/src/vlsu/shuffle_stage.sv \
  'PASS store transitions: C=%s,'

case "${1:-}" in
  --list) printf '%s\n' "${tests[@]}"; exit 0 ;;
  --help|'')
    printf 'Usage: run_tb_test.sh TEST_NAME | --list\n'
    printf 'TB_SIMULATOR=questa (default) or verilator selects the backend.\n'
    exit 0 ;;
esac
test=$1
if [[ ! $test =~ ^[a-z][a-z0-9_]*$ || -z ${rtl_sources[$test]+registered} ]]; then
  printf 'Unknown RTL testbench: %s (use --list)\n' "$test" >&2
  exit 2
fi
simulator=${TB_SIMULATOR:-questa}
case "$simulator" in
  questa|verilator) ;;
  *) printf 'Unknown TB_SIMULATOR: %s (expected questa or verilator)\n' "$simulator" >&2; exit 2 ;;
esac

build_dir=${BUILD_DIR:-$(mktemp -d "/tmp/araxl-${test}.XXXXXX")}
mkdir -p -- "$build_dir"
build_dir=$(cd -- "$build_dir" && pwd)
cd -- "$build_dir"
export TB_SIM_TIME=${TB_SIM_TIME:-2ms}
printf 'cd %q\nexport TB_SIM_TIME=%q\n' "$build_dir" "$TB_SIM_TIME" > "$build_dir/commands.log"

# Keep the exact commands with the test's generated files.
run_tool() {
  printf '%q ' "$@" >> "$build_dir/commands.log"
  printf '\n' >> "$build_dir/commands.log"
  "$@"
}

sources=(
  hardware/deps/common_cells/src/cf_math_pkg.sv
  hardware/deps/axi/src/axi_pkg.sv
)
if [[ ${package_groups[$test]} == ara ]]; then
  sources+=(
    hardware/deps/fpnew/src/fpnew_pkg.sv
    hardware/deps/ariane/core/include/cv64a6_imafdcv_sv39_config_pkg.sv
    hardware/deps/ariane/core/include/riscv_pkg.sv
    hardware/deps/ariane/common/local/rvfi/rvfi_pkg.sv
    hardware/deps/ariane/core/include/ariane_dm_pkg.sv
    hardware/deps/ariane/core/include/ariane_pkg.sv
    hardware/deps/ariane/core/include/acc_pkg.sv
    hardware/include/rvv_pkg.sv
    hardware/include/ara_pkg.sv
  )
fi
read -r -a rtl <<< "${rtl_sources[$test]}"
sources+=("${rtl[@]}" "hardware/tb/unit/${test}_tb.sv")
for i in "${!sources[@]}"; do sources[i]="$repo_root/${sources[i]}"; done

if [[ $simulator == questa ]]; then
  read -r -a questa_cmd <<< "${QUESTA_CMD:-questa-2021.3}"
  run_tool "${questa_cmd[@]}" vmap -c
  run_tool "${questa_cmd[@]}" vlib "$build_dir/work"
  run_tool "${questa_cmd[@]}" vmap -modelsimini "$build_dir/modelsim.ini" work "$build_dir/work"
  run_tool "${questa_cmd[@]}" vlog -modelsimini "$build_dir/modelsim.ini" -work work -sv \
    -timescale 1ns/1ps -suppress vlog-2583 -suppress vlog-13314 -suppress vlog-13233 \
    +define+VLEN=4096 +define+ARIANE_ACCELERATOR_PORT=1 +define+COMMON_CELLS_ASSERTS_OFF \
    "+incdir+$repo_root/hardware/deps/common_cells/include" \
    "+incdir+$repo_root/hardware/deps/axi/include" \
    -y "$repo_root/hardware/deps/common_cells/src" +libext+.sv \
    -l "$build_dir/compile.log" "${sources[@]}"

  cat > "$build_dir/run.tcl" <<'TCL'
onbreak {quit -code 1 -f}
onerror {quit -code 1 -f}
if {![info exists ::env(TB_LOG_ONLY)] || $::env(TB_LOG_ONLY) ne "1"} {log -r /*}
run {*}$::env(TB_SIM_TIME)
puts "TB_REGRESSION_TIMEOUT"
quit -code 124 -f
TCL

  wlf_args=()
  if [[ ${TB_LOG_ONLY:-0} == 1 ]]; then wlf_args+=(-wlfdeleteonquit); fi
  run_tool "${questa_cmd[@]}" vsim -c -modelsimini "$build_dir/modelsim.ini" -work work \
    -voptargs=+acc -onfinish exit -l "$build_dir/transcript" \
    -wlf "$build_dir/vsim.wlf" "${wlf_args[@]}" "work.${test}_tb" -do "$build_dir/run.tcl"
else
  verilator_cmd=${VERILATOR:-"$repo_root/install/verilator/bin/verilator"}
  if [[ ! -x $verilator_cmd ]]; then
    verilator_cmd=$(command -v "${VERILATOR:-verilator}")
  fi
  flags=(-Wno-fatal -Wno-WIDTH)
  defines=()
  if [[ ${package_groups[$test]} == ara ]]; then
    flags+=(-Wno-ASCRANGE -Wno-WIDTHCONCAT -Wno-CASEINCOMPLETE)
    defines+=(-DVLEN=4096 -DARIANE_ACCELERATOR_PORT=1 -DCOMMON_CELLS_ASSERTS_OFF)
  fi
  read -r -a extra_flags <<< "${verilator_flags[$test]}"
  run_tool "$verilator_cmd" --cc --exe --main --timing --assert \
    "${flags[@]}" "${extra_flags[@]}" \
    "${defines[@]}" \
    -I"$repo_root/hardware/deps/common_cells/include" \
    -I"$repo_root/hardware/deps/axi/include" \
    -y "$repo_root/hardware/deps/common_cells/src" \
    --top-module "${test}_tb" --Mdir "$build_dir" "${sources[@]}" \
    2>&1 | tee "$build_dir/compile.log"
  # Timing uses C++20 coroutines. An explicit compiler override must also
  # replace the installed Verilator's compiler-specific warning/tuning flags
  # (e.g. Clang's -fbracket-depth and -Qunused-arguments are invalid for GCC).
  build_args=(-C "$build_dir" -f "V${test}_tb.mk" -j "${JOBS:-4}" OBJCACHE=
              CFG_CXXFLAGS_COROUTINES=-std=c++20)
  if [[ -n ${CXX:-} ]]; then
    build_args+=("CXX=$CXX" "LINK=$CXX"
      "CFG_CXXFLAGS_NO_UNUSED=-Wno-unused-parameter -Wno-unused-variable -Wno-sign-compare -Wno-shadow"
      "CFG_CXXFLAGS_WEXTRA=-Wextra")
  fi
  run_tool make "${build_args[@]}" \
    2>&1 | tee -a "$build_dir/compile.log"
  run_tool timeout --kill-after=10s "${TIMEOUT:-60s}" "$build_dir/V${test}_tb" \
    2>&1 | tee "$build_dir/transcript"
fi

# Require completion for every instantiated configuration as well as exit 0.
# This also guards against an unexpected early $finish or assertion exit.
# Questa 2021.3 can exit 0 after $fatal when -onfinish exit is enabled.
if grep -Eq '\*\* (Error|Fatal|Failure):|%Error:|%Fatal:' "$build_dir/transcript"; then
  printf 'Simulation reported an error: %s\n' "$build_dir/transcript" >&2
  exit 1
fi
for clusters in 2 4 8 16; do
  printf -v marker "${pass_formats[$test]}" "$clusters"
  if ! grep -Fq "$marker" "$build_dir/transcript"; then
    printf 'Missing completion marker: %s\n' "$marker" >&2
    exit 1
  fi
done
if [[ -n ${extra_markers[$test]} ]]; then
  grep -Fq "${extra_markers[$test]}" "$build_dir/transcript"
fi
if [[ ${TB_LOG_ONLY:-0} != 1 ]]; then printf 'Testbench artifacts: %s\n' "$build_dir"; fi
