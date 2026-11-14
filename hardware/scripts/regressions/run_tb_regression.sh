#!/usr/bin/env bash
# Copyright 2026 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
simulator=${TB_SIMULATOR:-questa}
case "$simulator" in
  questa) simulator_name=QuestaSim ;;
  verilator) simulator_name=Verilator ;;
  *) printf 'Unknown TB_SIMULATOR: %s (expected questa or verilator)\n' "$simulator" >&2; exit 2 ;;
esac
build_root=${BUILD_DIR:-$(mktemp -d /tmp/araxl-tb-regression.XXXXXX)}
mkdir -p -- "$build_root"
build_root=$(cd -- "$build_root" && pwd)
run_dir="$build_root/$simulator"
mkdir -p -- "$run_dir"
log="$run_dir/run.log"
# Work libraries and generated sources are temporary; only run.log is retained.
scratch_dir=$(mktemp -d /tmp/araxl-tb-regression.XXXXXX)
trap 'rm -rf -- "$scratch_dir"' EXIT
exec 9>> "$log"
if ! flock -n 9; then
  printf 'Testbench suite already running: %s\n' "$log" >&2
  exit 1
fi
test_timeout=${TB_TIMEOUT:-300s}

test_list=$(bash "$script_dir/run_tb_test.sh" --list)
if [[ -z $test_list ]]; then
  printf 'No RTL testbenches registered\n' >&2
  exit 2
fi
mapfile -t tests <<< "$test_list"

failed_tests=()
printf 'Running %d RTL testbenches with %s (2/4/8/16 clusters)\n' "${#tests[@]}" "$simulator_name"
printf 'Log: %s\n' "$log"
printf '\n=== RTL testbench suite: %s ===\n' "$simulator_name" >> "$log"

for test in "${tests[@]}"; do
  test_dir="$scratch_dir/$test"
  mkdir -p -- "$test_dir"
  printf '\n=== %s ===\n' "$test" >> "$log"
  log_start=$(wc -l < "$log")
  # Bound compilation and simulation together for each registered test.
  if (
    cd -- "$test_dir" || exit 1
    BUILD_DIR="$test_dir/build" TB_LOG_ONLY=1 timeout --kill-after=10s "$test_timeout" \
      bash "$script_dir/run_tb_test.sh" "$test"
  ) >> "$log" 2>&1; then
    result=PASSED
    status=0
  else
    status=$?
    result=FAILED
    if [[ $status -eq 124 || $status -eq 137 ]]; then result=TIMEOUT; fi
    failed_tests+=("$test")
  fi
  color= reset=
  if [[ $result == PASSED ]]; then
    color=$'\033[92m'
    reset=$'\033[0m'
  fi
  printf '  %s%-7s%s %s (exit %d; %s)\n' "$color" "$result" "$reset" "$test" "$status" "$log"
  if [[ $result != PASSED ]]; then
    printf 'Last output from %s:\n' "$test"
    tail -n "+$((log_start + 1))" "$log" | tail -n 20
  fi
  printf 'RESULT\t%s\t%s\t%d\n' "$test" "$result" "$status" >> "$log"
done

printf 'Testbench log: %s\n' "$log"
if (( ${#failed_tests[@]} )); then
  printf 'FAILED RTL TESTBENCHES: %s\n' "${failed_tests[*]}"
  exit 1
fi
printf 'ALL %d RTL TESTBENCHES PASSED\n' "${#tests[@]}"
