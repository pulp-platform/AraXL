#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

"""Build and run application regressions with isolated simulator configurations."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile


def execute(command, cwd, log, timeout=None):
    with log.open("a") as output:
        try:
            process = subprocess.Popen(command, cwd=cwd, stdout=output,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        except OSError as error:
            output.write(str(error) + "\n")
            return 127
        try:
            return process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as interruption:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            if isinstance(interruption, KeyboardInterrupt):
                raise
            return 124


def record_command(path, command, cwd):
    with path.open("a") as output:
        output.write(f"cd {shlex.quote(str(cwd))}\n{shlex.join(command)}\n")


def build(args, directory, clusters, log):
    # Copy sources, including local edits, without any stale objects/binaries.
    # Application data generators and linker scripts write only in this copy.
    shutil.copytree(args.repo / "apps", directory / "apps",
                    ignore=shutil.ignore_patterns(".git", "bin", "*.o", "*.o.spike",
                                                  "*.dump", "__pycache__"),
                    symlinks=False)
    shutil.copytree(args.repo / "config", directory / "config")
    parameters = [f"config={args.config}", f"nr_lanes={args.lanes}",
                  f"nr_clusters={clusters}", f"vlen={args.vlen}"]
    applications = sorted(set(args.rvv + args.benchmarks))
    command = shlex.split(args.make) + ["--no-print-directory", "-C", str(directory / "apps"),
                                      f"ARA_DIR={args.repo}"] + parameters
    command += [f"bin/{app}" for app in applications]
    if applications:
        record_command(log, command, args.repo)
        if execute(command, args.repo, log, args.build_timeout):
            return False
    hardware = directory / "hardware"
    target = "verilate" if args.sim == "verilator" else "compile"
    command = shlex.split(args.make) + ["--no-print-directory", "-C", str(args.repo / "hardware"),
                                      target, f"buildpath={hardware}",
                                      f"veril_library={hardware / 'verilator'}",
                                      "library=work", "dpi_library=work-dpi",
                                      "top_level=ara_tb", "veril_top=ara_tb_verilator",
                                      "perfetto_trace=0", "trace=", "vcd_dump=0"] + parameters
    record_command(log, command, args.repo)
    if execute(command, args.repo, log, args.build_timeout):
        return False
    if args.sim == "questa" and not (hardware / f"compile_{args.config}.tcl").is_file():
        return False
    if args.sim == "verilator" and not (hardware / "verilator" / "Vara_tb_verilator").is_file():
        return False
    return True


def run_application(args, directory, suite, app, clusters, log):
    binary = directory / "apps" / "bin" / app
    if not binary.is_file():
        with log.open("a") as output:
            output.write(f"Missing binary: {binary}\n")
        print(f"  FAILED  {app} (missing binary; {log})", flush=True)
        return "FAILED", 1
    hardware = directory / "hardware"
    with tempfile.TemporaryDirectory(prefix="araxl-ci-sim-") as temporary:
        parent = Path(temporary)
        run = parent / "run"
        run.mkdir()
        # Existing scalar-core tracers have no disable option. Send their fixed
        # output paths to /dev/null; no trace data is written to disk. Vector
        # tracing is disabled when compiling new CI models as well.
        names = ["trace_hart_0.log", "trace_hart_0_commit.log", "trace_hart_00.dasm"]
        for cluster in range(clusters):
            names += [f"trace_vldu_c{cluster}.csv", f"trace_vstu_c{cluster}.csv"]
            names += [f"trace_mfpu_c{cluster}_l{lane}.csv" for lane in range(args.lanes)]
        for name in names:
            (run / name).symlink_to(os.devnull)
        for name in ("gold_results.txt", "id_results.txt"):
            (parent / name).symlink_to(os.devnull)
        if args.sim == "verilator":
            command = [str(hardware / "verilator" / "Vara_tb_verilator"),
                       "-c", str(args.verilator_max_cycles), "-l", f"ram,{binary},elf"]
        else:
            ini = run / "modelsim.ini"
            ini.write_text(f"[Library]\nwork = {hardware / 'work'}\n"
                           f"others = {hardware / 'modelsim.ini'}\n")
            tcl = run / "run.tcl"
            tcl.write_text("onbreak {quit -code 1 -f}\nonerror {quit -code 1 -f}\n"
                           + f"run {args.questa_sim_time}\n"
                           "puts \"CI_SIMULATION_TIMEOUT\"\nquit -code 124 -f\n")
            command = shlex.split(args.questa_cmd) + [
                "vsim", "-c", "-modelsimini", str(ini), "-work", "work",
                "-nolog", "-wlfdeleteonquit", "-wlf", str(run / "vsim.wlf"),
                "-sv_lib", str(hardware / "work-dpi" / "ara_dpi"),
                f"+PRELOAD={binary}", "-voptargs=+acc", "-suppress", "vsim-3009",
                "-suppress", "vopt-7033", "-onfinish", "exit", "work.ara_tb", "-do", str(tcl)]
        with log.open("a") as output:
            output.write(f"\n=== {suite}: {app} ===\n")
        record_command(log, command, run)
        offset = log.stat().st_size
        status = execute(command, run, log, args.timeout)
        # Check only this application's output. An earlier passing application
        # in the combined log must never make a later silent exit look successful.
        with log.open("rb") as output:
            output.seek(offset)
            text = output.read().decode(errors="replace")
    success = re.search(r"Core Test \*\*\* SUCCESS \*\*\* \(tohost = 0\)", text)
    errors = re.search(r"\*\* (?:Error|Fatal|Failure):|%Error:|%Fatal:", text)
    if status in (124, 137) or "Simulation timeout of" in text:
        result = "TIMEOUT"
    else:
        result = "PASSED" if status == 0 and success and not errors else "FAILED"
    label = f"\033[92m{result:7}\033[0m" if result == "PASSED" else f"{result:7}"
    print(f"  {label} {app} (exit {status}; {log})", flush=True)
    return result, status


def reused_workspace(args, root, log, metadata):
    # A deliberate ci-build keeps a reusable model in /tmp, with its location
    # recorded in run.log rather than extra metadata files in the build folder.
    if log.exists():
        for line in reversed(log.read_text(errors="replace").splitlines()):
            if line.startswith("CI_WORKSPACE "):
                saved = json.loads(line[len("CI_WORKSPACE "):])
                if saved["configuration"] == metadata and Path(saved["path"]).is_dir():
                    return Path(saved["path"])
    # Allow reuse of a completed build from the previous CI directory layout.
    legacy = root / args.sim / args.config / f"c{metadata['clusters']}-l{args.lanes}-v{args.vlen}"
    manifest = legacy / "configuration.json"
    if manifest.is_file() and json.loads(manifest.read_text()) == metadata:
        return legacy
    return None


def result_line(log, simulator, clusters, suite, app, result, status):
    with log.open("a") as output:
        output.write(f"RESULT\t{simulator}\t{clusters}\t{suite}\t{app}\t{result}\t{status}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("all", "build", "rvv", "benchmarks"), required=True)
    parser.add_argument("--sim", choices=("verilator", "questa"), required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--clusters", nargs="+", type=int, required=True)
    parser.add_argument("--lanes", type=int, required=True)
    parser.add_argument("--vlen", type=int, required=True)
    parser.add_argument("--make", default="make")
    parser.add_argument("--questa-cmd", default="questa-2021.3")
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--build-timeout", type=float, default=1800)
    parser.add_argument("--questa-sim-time", default="100ms",
                        help="Questa simulated-time limit per application (default: 100ms)")
    parser.add_argument("--verilator-max-cycles", type=int, default=100000000,
                        help="Verilator clock-cycle limit per application (default: 100000000)")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--rvv", nargs="*", default=[])
    parser.add_argument("--benchmarks", nargs="*", default=[])
    args = parser.parse_args()
    if any(c not in (2, 4, 8, 16) for c in args.clusters):
        parser.error("cluster configurations must be 2, 4, 8, or 16")
    if len(set(args.clusters)) != len(args.clusters):
        parser.error("cluster configurations must not be repeated")
    if args.timeout <= 0 or args.build_timeout <= 0:
        parser.error("timeouts must be positive")
    if args.sim == "verilator" and args.verilator_max_cycles <= 0:
        parser.error("verilator-max-cycles must be positive")
    if args.sim == "questa" and not re.fullmatch(
            r"[0-9]+(?:\.[0-9]+)?\s*(?:fs|ps|ns|us|ms|sec)", args.questa_sim_time):
        parser.error("questa-sim-time must be a positive time such as 100ms")
    if args.stage == "rvv":
        args.benchmarks = []
    elif args.stage == "benchmarks":
        args.rvv = []
    # Derive paths from this script, independently of the caller's directory.
    args.repo = Path(__file__).resolve().parents[3]
    root = args.repo / "hardware" / "build" / "ci"
    root.mkdir(parents=True, exist_ok=True)
    print(f"CI simulator: {args.sim}; logs: {root}", flush=True)
    failed = []
    for clusters in args.clusters:
        name = f"{args.sim}-{args.config}-c{clusters}-l{args.lanes}-v{args.vlen}"
        output_dir = root / name
        output_dir.mkdir(parents=True, exist_ok=True)
        log = output_dir / "run.log"
        metadata = {"simulator": args.sim, "config": args.config, "clusters": clusters,
                    "lanes": args.lanes, "vlen": args.vlen}
        # Lock the log itself: parallel runs of the same configuration must not
        # mix output or contaminate each other's application completion checks.
        with log.open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print(f"  FAILED  configuration already running: {output_dir}", flush=True)
                failed.append(name)
                continue
            directory = reused_workspace(args, root, log, metadata) if args.skip_build else None
            if args.skip_build and directory is None:
                parser.error(f"no matching compiled configuration: {output_dir}")
            temporary = directory is None
            if temporary:
                directory = Path(tempfile.mkdtemp(prefix=f"araxl-ci-{args.sim}-c{clusters}-"))
            keep = False
            try:
                with log.open("a") as output:
                    output.write(f"\n=== CI {args.stage}: {name} ===\n")
                if not args.skip_build:
                    print(f"Building {name} ({log})", flush=True)
                    try:
                        built = build(args, directory, clusters, log)
                    except (OSError, shutil.Error) as error:
                        with log.open("a") as output:
                            output.write(str(error) + "\n")
                        built = False
                    if not built:
                        failed.append(f"build-{clusters}")
                        result_line(log, args.sim, clusters, "build", "-", "FAILED", 1)
                        print(f"  FAILED  build-{clusters}: {log}", flush=True)
                        continue
                    if args.stage == "build":
                        keep = True
                        with log.open("a") as output:
                            output.write("CI_WORKSPACE " + json.dumps({
                                "configuration": metadata, "path": str(directory),
                            }) + "\n")
                        print(f"Compiled model retained for skip_build=1; location recorded in {log}", flush=True)
                stages = ("rvv", "benchmarks") if args.stage == "all" else (args.stage,)
                for stage in stages:
                    if stage == "build":
                        continue
                    applications = args.rvv if stage == "rvv" else args.benchmarks
                    print(f"{stage}: {args.sim}, {clusters} clusters", flush=True)
                    for app in applications:
                        result, status = run_application(args, directory, stage, app, clusters, log)
                        result_line(log, args.sim, clusters, stage, app, result, status)
                        if result != "PASSED":
                            failed.append(f"{stage}-{clusters}-{app}")
            finally:
                if temporary and not keep:
                    shutil.rmtree(directory)
    if failed:
        print("FAILED CI CHECKS: " + " ".join(failed), flush=True)
        return 1
    print("ALL CI CHECKS PASSED", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
