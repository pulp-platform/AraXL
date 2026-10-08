#!/usr/bin/env python3
"""
Multi-core vs Single-core Benchmark Utilization Comparison.

Compares the utilization of the 4-core configuration against the 1-core configuration
across all benchmarks for Bytes/Lane swept from 8 to 64 B/lane.

- 4-core logs are identified by '*4core*'.
- 1-core logs are identified by '*1core*', falling back to default single-core logs
  without 'core' in the filename.
- For 'streamcluster', the highest reported 'pgain' utilization is used.
- 'canneal' utilization is computed as sum(fpu-cycles) / sw-cycles.
- 'spmv' uses its already-reported 'util' value.
- Both 'canneal' and 'spmv' are only printed in the summary table, never plotted.
"""

import argparse
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# Shared qualitative palette, reused (in the same order) by plot_benchmark.py
# and plot_latency_tolerance.py so all three scripts look consistent.
PALETTE = [
    "#2A9D8F", "#E76F51", "#E9C46A", "#264653",
    "#F4A261", "#6A4C93", "#1982C4", "#8AC926",
]

BYTES_SWEEP = [8, 16, 32, 64, 128, 256, 512]
EXCLUDED_BENCHMARKS = set()
# These benchmarks use sum(fpu-cycles) / sw-cycles for utilization.
FPU_SUM_BENCHMARKS = {"canneal", "_canneal"}
# These benchmarks are print-only: never plotted, individually or in the summary.
PRINT_ONLY_BENCHMARKS = {"canneal", "_canneal", "spmv", "_spmv"}
# Average vector length (fan_locs count x2 for x/y coords) from 100.nets, per
# apps/riscv-vectorized-benchmark-suite/_canneal/script/stats.py.
CANNEAL_AVG_VECTOR_LENGTH = 19.8
BENCHMARK_ORDER = [
    "fmatmul",
    "fconv2d",
    "axpy",
    "dotp",
    "exp",
    "softmax",
    "blackscholes",
    "swaptions",
    "pathfinder",
    "streamcluster",
    "lavamd",
]
DISPLAY_NAME_MAP = {
    "fdotproduct": "dotp",
    "lavaMD": "lavamd",
    "_lavaMD": "lavamd",
    "_fdotproduct": "dotp",
}

UTIL_RE = re.compile(
    r"\butil\s*:\s*(?P<util>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*%?",
    re.IGNORECASE,
)
PGAIN_UTIL_RE = re.compile(
    r"pgain.*?util\s*:\s*(?P<util>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*%?",
    re.IGNORECASE,
)
FPU_CYCLES_RE = re.compile(
    r"\[fpu-cycles\]\s*:\s*(?P<val>\d+)",
    re.IGNORECASE,
)
SW_CYCLES_RE = re.compile(
    r"\[sw-cycles\]\s*:\s*(?P<val>\d+)",
    re.IGNORECASE,
)
NUM_CORES_RE = re.compile(r"num_cores\s*:\s*(?P<val>\d+)", re.IGNORECASE)
NUM_LANES_RE = re.compile(r"Lanes\s*=\s*(?P<val>\d+)", re.IGNORECASE)
NUM_CLUSTERS_RE = re.compile(r"Clusters\s*=\s*(?P<val>\d+)", re.IGNORECASE)


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent if script_dir.name == "scripts" else script_dir
    default_logs_dir = repo_root / "apps" / "logs"
    default_plots_dir = repo_root / "plots"

    parser = argparse.ArgumentParser(
        description="Compare 4-core vs 1-core benchmark utilization across 8 to 512 B/lane."
    )
    parser.add_argument(
        "--logs-dir",
        type=Path,
        default=default_logs_dir,
        help=f"Path to the benchmark logs directory (default: {default_logs_dir})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory to save individual per-benchmark plots (default: each benchmark's logs directory)",
    )
    parser.add_argument(
        "--output-summary",
        type=Path,
        default=None,
        help=f"Path to save the combined summary plot (default: {default_plots_dir / 'multicore_utilization_comparison.svg'})",
    )
    parser.add_argument(
        "--bytes",
        type=int,
        nargs="+",
        default=BYTES_SWEEP,
        help=f"Bytes/Lane values to sweep (default: {BYTES_SWEEP})",
    )
    parser.add_argument(
        "--font-size",
        type=int,
        default=20,
        help="Base font size for axis labels (default: 12)",
    )
    return parser.parse_args()


def read_utilization(log_file: Path, is_streamcluster: bool = False) -> float | None:
    """Extract utilization percentage from a log file.
    
    For streamcluster, extracts the maximum reported pgain utilization.
    For other benchmarks, extracts the last reported utilization.
    """
    if not log_file.is_file():
        return None

    try:
        content = log_file.read_text(encoding="utf-8", errors="replace")
    except Exception as e:
        print(f"Warning: Failed to read {log_file}: {e}", file=sys.stderr)
        return None

    if is_streamcluster:
        pgain_values = []
        for match in PGAIN_UTIL_RE.finditer(content):
            try:
                pgain_values.append(float(match.group("util")))
            except ValueError:
                pass
        return max(pgain_values) if pgain_values else None
    else:
        last_util = None
        for match in UTIL_RE.finditer(content):
            try:
                last_util = float(match.group("util"))
            except ValueError:
                pass
        return last_util


def read_fpu_sum_utilization(log_file: Path) -> float | None:
    """Compute utilization as sum(fpu-cycles) / (sw-cycles * num_cores * lanes * clusters)."""
    if not log_file.is_file():
        return None

    try:
        content = log_file.read_text(encoding="utf-8", errors="replace")
    except Exception as e:
        print(f"Warning: Failed to read {log_file}: {e}", file=sys.stderr)
        return None

    fpu_sum = sum(int(m.group("val")) for m in FPU_CYCLES_RE.finditer(content))
    sw_match = SW_CYCLES_RE.search(content)
    if sw_match is None:
        return None

    sw_cycles = int(sw_match.group("val"))
    if sw_cycles == 0:
        return None

    num_cores_match = NUM_CORES_RE.search(content)
    num_lanes_match = NUM_LANES_RE.search(content)
    num_clusters_match = NUM_CLUSTERS_RE.search(content)
    num_cores = int(num_cores_match.group("val")) if num_cores_match else 1
    num_lanes = int(num_lanes_match.group("val")) if num_lanes_match else 1
    num_clusters = int(num_clusters_match.group("val")) if num_clusters_match else 1
    total_lanes = num_cores * num_lanes * num_clusters

    if num_cores == 1:
        # Single-core (64-lane): 32-bit x/y coords only occupy half of each 64-bit lane.
        return fpu_sum / (sw_cycles * total_lanes * 2) * 100

    # Multi-core: scale by how much of the 32-bit-wide, 64-lane datapath the
    # average vector (fan_locs length) actually fills.
    vector_length_factor = CANNEAL_AVG_VECTOR_LENGTH / 32
    return fpu_sum / (sw_cycles * total_lanes) * vector_length_factor * 100


def find_matching_log(
    files: list[Path],
    bpl: int,
    is_4core: bool,
) -> Path | None:
    """Find the best matching log file for a given B/lane and core configuration."""
    bpl_pattern = re.compile(rf"_{bpl}B[_\.]")
    
    if is_4core:
        matching = [f for f in files if "4core" in f.name and bpl_pattern.search(f.name)]
        if not matching:
            return None
        # Prefer baseline 0mem if available
        matching_0mem = [f for f in matching if "0mem" in f.name]
        return matching_0mem[0] if matching_0mem else matching[0]
    else:
        # 1. Look for explicit 1core log
        matching_1core = [f for f in files if "1core" in f.name and bpl_pattern.search(f.name)]
        if matching_1core:
            matching_0mem = [f for f in matching_1core if "0mem" in f.name]
            return matching_0mem[0] if matching_0mem else matching_1core[0]

        # 2. Fall back to default log without any 'core' in filename
        matching_default = [
            f for f in files
            if "core" not in f.name
            and bpl_pattern.search(f.name)
            and ("0mem" in f.name or f.name.endswith(".log"))
        ]
        if not matching_default:
            return None

        # Prefer 16C (16 clusters = 16 cores baseline) if available, then 0mem
        matching_16c = [f for f in matching_default if "16C" in f.name and "0mem" in f.name]
        if matching_16c:
            return matching_16c[0]
        matching_0mem = [f for f in matching_default if "0mem" in f.name]
        if matching_0mem:
            return matching_0mem[0]
        return matching_default[0]


def collect_benchmark_data(logs_dir: Path, bytes_sweep: list[int]):
    """Collect 1-core and 4-core utilization data for all benchmarks."""
    if not logs_dir.is_dir():
        print(f"Error: Logs directory '{logs_dir}' not found.", file=sys.stderr)
        return {}

    benchmark_data = {}

    for bench_dir in sorted(logs_dir.iterdir()):
        if not bench_dir.is_dir():
            continue

        bench_name = bench_dir.name
        if bench_name in EXCLUDED_BENCHMARKS or bench_name.lstrip("_") in EXCLUDED_BENCHMARKS:
            continue

        is_sc = "streamcluster" in bench_name.lower()
        is_fpu_sum = bench_name in FPU_SUM_BENCHMARKS
        log_files = sorted(bench_dir.glob("*.log"))
        if not log_files:
            continue

        data_1core = {}
        data_4core = {}
        files_1core = {}
        files_4core = {}

        for bpl in bytes_sweep:
            # 4-core lookup
            f_4c = find_matching_log(log_files, bpl, is_4core=True)
            if f_4c:
                u_4c = (
                    read_fpu_sum_utilization(f_4c)
                    if is_fpu_sum
                    else read_utilization(f_4c, is_streamcluster=is_sc)
                )
                if u_4c is not None:
                    data_4core[bpl] = u_4c
                    files_4core[bpl] = f_4c.name

            # 1-core lookup
            f_1c = find_matching_log(log_files, bpl, is_4core=False)
            if f_1c:
                u_1c = (
                    read_fpu_sum_utilization(f_1c)
                    if is_fpu_sum
                    else read_utilization(f_1c, is_streamcluster=is_sc)
                )
                if u_1c is not None:
                    data_1core[bpl] = u_1c
                    files_1core[bpl] = f_1c.name

        if data_1core or data_4core:
            benchmark_data[bench_name] = {
                "1core": data_1core,
                "4core": data_4core,
                "files_1core": files_1core,
                "files_4core": files_4core,
                "dir": bench_dir,
            }

    return benchmark_data


def print_data_table(benchmark_data: dict, bytes_sweep: list[int]):
    """Print a formatted summary table of all benchmark utilizations."""
    print("=" * 78)
    print(f"{'Benchmark':<18} {'B/Lane':<8} {'1-Core Util (%)':<18} {'4-Core Util (%)':<18} {'Ratio (4C/1C)':<14}")
    print("=" * 78)

    for bench_name, d in sorted(benchmark_data.items()):
        display_name = bench_name.lstrip("_")
        for bpl in bytes_sweep:
            val_1c = d["1core"].get(bpl)
            val_4c = d["4core"].get(bpl)

            str_1c = f"{val_1c:.2f}%" if val_1c is not None else "N/A"
            str_4c = f"{val_4c:.2f}%" if val_4c is not None else "N/A"
            if val_1c is not None and val_4c is not None and val_1c > 0:
                ratio = f"{val_4c / val_1c:.2f}x"
            else:
                ratio = "N/A"

            print(f"{display_name:<18} {bpl:<8} {str_1c:<18} {str_4c:<18} {ratio:<14}")
            display_name = ""  # Print benchmark name only on first row
        print("-" * 78)


def format_byte_tick(value: float) -> str:
    """Format axis values like 32768 as 32K instead of 32768."""
    value_int = int(value)
    if value_int >= 1024 and value_int % 1024 == 0:
        return f"{value_int // 1024}K"
    return str(value_int)


def display_benchmark_name(bench_name: str) -> str:
    """Map benchmark directory names to the requested display names."""
    base_name = bench_name.lstrip("_")
    return DISPLAY_NAME_MAP.get(base_name, DISPLAY_NAME_MAP.get(bench_name, base_name.lower()))


def plot_single_benchmark(bench_name: str, data: dict, output_path: Path, font_size: int = 12):
    """Generate and save a 2-curve comparison plot for a single benchmark."""
    fig, ax = plt.subplots(figsize=(7, 5), dpi=300)

    x_1c = sorted(data["1core"].keys())
    y_1c = [data["1core"][b] for b in x_1c]

    x_4c = sorted(data["4core"].keys())
    y_4c = [data["4core"][b] for b in x_4c]

    x_1c_scaled = [b * 64 for b in x_1c]
    x_4c_scaled = [b * 64 for b in x_4c]

    # Plot 1-core curve
    if x_1c:
        ax.plot(
            x_1c_scaled,
            y_1c,
            color=PALETTE[0],
            marker="o",
            linestyle="-",
            linewidth=2.2,
            markersize=7,
            label="1 Core Configuration",
        )

    # Plot 4-core curve
    if x_4c:
        ax.plot(
            x_4c_scaled,
            y_4c,
            color=PALETTE[1],
            marker="s",
            linestyle="--",
            linewidth=2.2,
            markersize=7,
            label="4 Core Configuration",
        )

    ax.set_xlabel("vector length (Bytes)", fontsize=font_size, fontweight="bold")
    ax.set_ylabel("Utilization (%)", fontsize=font_size, fontweight="bold")

    ax.set_xscale("log", base=2)
    all_x = sorted(set(x_1c_scaled + x_4c_scaled))
    if all_x:
        ax.set_xticks(all_x)
        ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: format_byte_tick(v)))

    ax.set_ylim(0, 100)

    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend(frameon=True, fontsize=font_size - 1, loc="best")
    plt.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved plot: {output_path}")


def plot_combined_summary(benchmark_data: dict, output_path: Path, font_size: int = 12):
    """Generate a combined multi-panel figure showing all benchmarks."""
    sub_font_size = font_size - 2
    sub_tick_font_size = sub_font_size - 2
    valid_benchmarks = [
        b for b, d in benchmark_data.items()
        if (d["1core"] or d["4core"]) and b not in PRINT_ONLY_BENCHMARKS
    ]
    ordered_benchmarks = []
    seen = set()
    for desired_name in BENCHMARK_ORDER:
        for bench_name in valid_benchmarks:
            if bench_name in seen:
                continue
            if display_benchmark_name(bench_name) == desired_name:
                ordered_benchmarks.append(bench_name)
                seen.add(bench_name)
    for bench_name in valid_benchmarks:
        if bench_name not in seen:
            ordered_benchmarks.append(bench_name)

    n_benchmarks = len(ordered_benchmarks)
    if n_benchmarks == 0:
        return

    rows = 2
    cols = max(1, (n_benchmarks + 1) // 2)

    fig, axes = plt.subplots(
        rows,
        cols,
        figsize=(4.5 * cols, 3.8 * rows),
        squeeze=False,
        dpi=300,
    )

    for i, bench_name in enumerate(ordered_benchmarks):
        row = i // cols
        col = i % cols
        ax = axes[row][col]
        d = benchmark_data[bench_name]

        x_1c = sorted(d["1core"].keys())
        y_1c = [d["1core"][b] for b in x_1c]

        x_4c = sorted(d["4core"].keys())
        y_4c = [d["4core"][b] for b in x_4c]

        x_1c_scaled = [b * 64 for b in x_1c]
        x_4c_scaled = [b * 64 for b in x_4c]

        if x_1c:
            ax.plot(
                x_1c_scaled,
                y_1c,
                color=PALETTE[0],
                marker="o",
                linestyle="-",
                linewidth=1.8,
                markersize=6,
                label="1 Core",
            )
        if x_4c:
            ax.plot(
                x_4c_scaled,
                y_4c,
                color=PALETTE[1],
                marker="s",
                linestyle="--",
                linewidth=1.8,
                markersize=6,
                label="4 Core",
            )

        ax.set_title(display_benchmark_name(bench_name), fontsize=sub_font_size, fontweight="bold")
        ax.set_xlabel("vector length (Bytes)", fontsize=sub_font_size, fontweight="bold")
        ax.set_ylabel("Utilization (%)", fontsize=sub_font_size, fontweight="bold")
        ax.set_xscale("log", base=2)

        all_x = sorted(set(x_1c_scaled + x_4c_scaled))
        if all_x:
            ax.set_xticks(all_x)
            ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: format_byte_tick(v)))
        ax.tick_params(axis="both", labelsize=sub_tick_font_size)

        ax.set_ylim(0, 100)

        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(frameon=True, fontsize=sub_font_size - 1, loc="best")

    for i in range(n_benchmarks, rows * cols):
        row = i // cols
        col = i % cols
        fig.delaxes(axes[row][col])

    plt.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved combined summary plot: {output_path}")


def main():
    args = parse_args()
    data = collect_benchmark_data(args.logs_dir, args.bytes)

    if not data:
        print("No matching benchmark log data found.", file=sys.stderr)
        sys.exit(1)

    print_data_table(data, args.bytes)

    # Plot individual benchmarks
    for bench_name, d in data.items():
        if not d["1core"] and not d["4core"]:
            continue
        if bench_name in PRINT_ONLY_BENCHMARKS:
            continue
        if args.output_dir:
            out_file = args.output_dir / f"{bench_name.lstrip('_')}_multicore_utilization.svg"
        else:
            out_file = d["dir"] / f"{bench_name.lstrip('_')}_multicore_utilization.svg"
        plot_single_benchmark(bench_name, d, out_file, font_size=args.font_size)

    # Plot combined summary
    repo_root = Path(__file__).resolve().parent
    if repo_root.name == "scripts":
        repo_root = repo_root.parent
    summary_out = args.output_summary or (repo_root / "plots" / "multicore_utilization_comparison.svg")
    plot_combined_summary(data, summary_out, font_size=args.font_size)


if __name__ == "__main__":
    main()
