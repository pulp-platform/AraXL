#!/usr/bin/env python3
"""
Benchmark visualization script
Plots FPU utilization vs Bytes/Lane for different AraXL cluster configurations.
Log files are read from apps/logs*/<benchmark>/ with naming like:
{L}L_{C}C_{B}B_0mem.log or {L}L_{C}C_{core}core_{B}B_0mem.log
Only logs without an explicit multi-core suffix are plotted.
"""

import argparse
import os
import re
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

APPS_LOGS_DIR = os.path.join(os.path.dirname(__file__), '..', 'apps', 'logs')
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PLOTS_DIR = os.path.join(REPO_ROOT, 'plots')

_parser = argparse.ArgumentParser(description=__doc__)
_parser.add_argument(
    "--font-size",
    type=int,
    default=20,
    help="Base font size for axis labels (default: 12)",
)
_parser.add_argument(
    "--tick-font-size",
    type=int,
    default=20,
    help="Font size for x/y tick values (default: font-size - 2)",
)
args = _parser.parse_args()
FONT_SIZE = args.font_size
TICK_FONT_SIZE = args.tick_font_size if args.tick_font_size is not None else FONT_SIZE-4

# These benchmarks are print-only: never plotted, individually or in the combined summary.
PRINT_ONLY_BENCHMARKS = {"canneal", "_canneal", "spmv", "_spmv"}

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


def display_benchmark_name(bench_name):
    """Map a benchmark directory name to its display name (no underscore)."""
    base_name = bench_name.lstrip("_")
    return DISPLAY_NAME_MAP.get(base_name, DISPLAY_NAME_MAP.get(bench_name, base_name.lower()))


def ordered_benchmarks(all_data):
    """Return benchmark keys in the same order as plot_muticore.py."""
    ordered = []
    seen = set()
    for desired_name in BENCHMARK_ORDER:
        for bench_name in all_data:
            if bench_name in seen:
                continue
            if display_benchmark_name(bench_name) == desired_name:
                ordered.append(bench_name)
                seen.add(bench_name)
    for bench_name in all_data:
        if bench_name not in seen:
            ordered.append(bench_name)
    return ordered


def discover_benchmarks(logs_root):
    """Return benchmark directories that contain at least one log file."""
    if not os.path.isdir(logs_root):
        return []
    benchmarks = []
    for entry in sorted(os.listdir(logs_root)):
        bench_dir = os.path.join(logs_root, entry)
        if not os.path.isdir(bench_dir):
            continue
        if any(fname.endswith('.log') for fname in os.listdir(bench_dir)):
            benchmarks.append(entry)
    return benchmarks

def parse_log_utilization(filepath):
    """Extract the final utilization percentage from a log file."""
    util = None
    with open(filepath, 'r') as f:
        lines = f.readlines()
    for line in reversed(lines):
        m = re.search(r'util:([\d.]+)%', line)
        if m:
            util = float(m.group(1))
            break
    return util

def parse_canneal_counters(filepath):
    """Return (sw_cycles, total_lane_ops) for a _canneal log file."""
    sw_cycles = None
    total_lane_ops = 0
    with open(filepath, 'r') as f:
        for line in f:
            m = re.search(r'\[sw-cycles\][=:]\s*(\d+)', line)
            if m:
                sw_cycles = int(m.group(1))
            m2 = re.search(r'\[fpu-cycles\]\s*:\s*(\d+)', line)
            if m2:
                total_lane_ops += int(m2.group(1))
    if sw_cycles is not None and total_lane_ops:
        return sw_cycles, total_lane_ops
    return None, None

def get_canneal_reference_total_ops(logs_dir):
    """Use the 16C _canneal log as the fixed 32-bit op count."""
    candidates = []
    for fname in sorted(os.listdir(logs_dir)):
        if not fname.endswith('.log'):
            continue
        parsed = parse_filename(fname)
        if parsed is None:
            continue
        config, bytes_per_lane = parsed
        lanes, clusters, cores = config
        if lanes == 4 and clusters == 16 and cores is None:
            _, total_lane_ops = parse_canneal_counters(os.path.join(logs_dir, fname))
            if total_lane_ops is not None:
                candidates.append((bytes_per_lane, fname, total_lane_ops))
    if not candidates:
        return None, None
    _, fname, total_lane_ops = max(candidates)
    return total_lane_ops, fname

def parse_spmv_data(filepath):
    """Return (sw_cycles, avg_fpu_cycles, util_pct) for a _spmv log file."""
    sw_cycles = None
    util_pct = None
    fpu_cycles_list = []
    with open(filepath, 'r') as f:
        for line in f:
            m = re.search(r'\[sw-cycles\]:(\d+)\s+util:([\d.]+)%', line)
            if m:
                sw_cycles = int(m.group(1))
                util_pct = float(m.group(2))
            m2 = re.search(r'\[fpu-cycles\]\s*:\s*(\d+)', line)
            if m2:
                fpu_cycles_list.append(int(m2.group(1)))
    if sw_cycles is not None and fpu_cycles_list:
        avg_fpu = sum(fpu_cycles_list) / len(fpu_cycles_list)
        return sw_cycles, avg_fpu, util_pct
    return None, None, None

def parse_filename(filename):
    """Parse benchmark log filenames into a config label and bytes/lane."""
    m = re.match(r'(\d+)L_(\d+)C_(?:(\d+)core_)?(\d+)B_\d+mem\.log', filename)
    if m:
        lanes = int(m.group(1))
        clusters = int(m.group(2))
        cores = int(m.group(3)) if m.group(3) else None
        bytes_per_lane = int(m.group(4))
        config = (lanes, clusters, cores)
        return config, bytes_per_lane
    return None


def format_config(config):
    lanes, clusters, cores = config
    if cores is None:
        return f'{lanes}L_{clusters}C'
    return f'{lanes}L_{clusters}C_{cores}core'


def plot_combined_summary(all_data, output_path, cluster_colors, core_markers, font_size=12, tick_font_size=None):
    """Generate a combined multi-panel figure showing all benchmarks."""
    sub_font_size = font_size - 2
    sub_tick_font_size = (tick_font_size if tick_font_size is not None else font_size - 2) - 2
    benchmarks = [b for b in ordered_benchmarks(all_data) if b not in PRINT_ONLY_BENCHMARKS]
    n_benchmarks = len(benchmarks)
    if n_benchmarks == 0:
        return

    rows = 2
    cols = max(1, (n_benchmarks + 1) // 2)

    # Shared color/marker mapping across panels so a given cluster/core count
    # always looks the same regardless of which benchmark panel it's in.
    all_clusters = sorted({config[1] for data in all_data.values() for config in data})
    all_cores = sorted({
        config[2] if config[2] is not None else 1
        for data in all_data.values() for config in data
    })
    color_by_cluster = {
        clusters: cluster_colors[i % len(cluster_colors)]
        for i, clusters in enumerate(all_clusters)
    }
    marker_by_cores = {
        cores: core_markers[i % len(core_markers)]
        for i, cores in enumerate(all_cores)
    }

    fig, axes = plt.subplots(
        rows, cols, figsize=(4.5 * cols, 3.8 * rows), squeeze=False, dpi=300,
    )

    for i, benchmark in enumerate(benchmarks):
        row, col = divmod(i, cols)
        ax = axes[row][col]
        data = all_data[benchmark]
        all_bpl_vals = sorted({bpl for config_data in data.values() for bpl in config_data})

        for config in sorted(data):
            _, clusters, cores = config
            cores = cores if cores is not None else 1
            bpl_vals = sorted(data[config])
            util_vals = [data[config][b] for b in bpl_vals]
            ax.plot(
                bpl_vals, util_vals,
                color=color_by_cluster[clusters],
                marker=marker_by_cores[cores],
                linewidth=1.8, markersize=6,
                label=format_config(config),
            )

        ax.set_title(display_benchmark_name(benchmark), fontsize=sub_font_size, fontweight='bold')
        ax.set_xlabel('Bytes / Lane', fontsize=sub_font_size, fontweight='bold')
        ax.set_ylabel('FPU Utilization (%)', fontsize=sub_font_size, fontweight='bold')
        ax.set_xscale('log', base=2)
        ax.set_xticks(all_bpl_vals)
        ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))
        ax.tick_params(axis='both', labelsize=sub_tick_font_size)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_ylim(0, 105)
        ax.legend(frameon=True, fontsize=sub_tick_font_size, loc="best")

    for i in range(n_benchmarks, rows * cols):
        row, col = divmod(i, cols)
        fig.delaxes(axes[row][col])

    plt.tight_layout()

    plt.savefig(output_path, format='svg', bbox_inches='tight')
    print(f"Combined summary plot saved to {output_path}")
    plt.close(fig)

# Collect data per benchmark: {benchmark: {config: {bytes: util}}}
all_data = {}

for benchmark in discover_benchmarks(APPS_LOGS_DIR):
    logs_dir = os.path.join(APPS_LOGS_DIR, benchmark)
    data = defaultdict(dict)
    canneal_reference_ops = None
    if benchmark == '_canneal':
        canneal_reference_ops, canneal_reference_log = get_canneal_reference_total_ops(logs_dir)
        if canneal_reference_ops is None:
            print("WARNING: no 16C reference total ops found for _canneal")
    for fname in os.listdir(logs_dir):
        parsed = parse_filename(fname)
        if parsed is None:
            continue
        config, bytes_per_lane = parsed
        if config[2] is not None:
            continue
        fpath = os.path.join(logs_dir, fname)
        if benchmark == '_canneal' and canneal_reference_ops is not None:
            sw_cycles, _ = parse_canneal_counters(fpath)
            lanes, clusters, cores = config
            total_nr_lanes = lanes * clusters * (cores if cores is not None else 1)
            util = (canneal_reference_ops / (sw_cycles * total_nr_lanes * 2.0)) * 100.0 if sw_cycles else None
        else:
            util = parse_log_utilization(fpath)
        if util is not None:
            data[config][bytes_per_lane] = util
        else:
            print(f"WARNING: no util found in {benchmark}/{fname}")
    if data:
        all_data[benchmark] = data

if not all_data:
    print("No data found for any benchmark.")
    exit(1)

# Print collected data
for benchmark, data in all_data.items():
    print(f"\n--- {benchmark} ---")
    print(f"{'Config':<12} {'Bytes/Lane':<12} {'Utilization (%)'}")
    print("-" * 38)
    for config in sorted(data):
        for bpl in sorted(data[config]):
            print(f"{format_config(config):<12} {bpl:<12} {data[config][bpl]:.3f}%")

# Plot: one figure per benchmark, saved as SVG in the respective logs folder
# Shared qualitative palette, same order as plot_muticore.py's PALETTE and
# plot_latency_tolerance.py's palette, so all three scripts look consistent.
cluster_colors = ['#2A9D8F', '#E76F51', '#E9C46A', '#264653', '#F4A261', '#6A4C93', '#1982C4', '#8AC926']
core_markers = ['o', 's', '^', 'D', 'v', 'P', 'X', '*']

for benchmark in ordered_benchmarks(all_data):
    if benchmark in PRINT_ONLY_BENCHMARKS:
        continue
    data = all_data[benchmark]
    all_bpl_vals = sorted({bpl for config_data in data.values() for bpl in config_data})
    fig, ax = plt.subplots(figsize=(8, 6))
    cluster_values = sorted({config[1] for config in data})
    core_counts = sorted({config[2] if config[2] is not None else 1 for config in data})
    color_by_cluster = {
        clusters: cluster_colors[i % len(cluster_colors)]
        for i, clusters in enumerate(cluster_values)
    }
    marker_by_cores = {
        cores: core_markers[i % len(core_markers)]
        for i, cores in enumerate(core_counts)
    }

    for config in sorted(data):
        _, clusters, cores = config
        cores = cores if cores is not None else 1
        bpl_vals = sorted(data[config])
        util_vals = [data[config][b] for b in bpl_vals]
        label = format_config(config)
        ax.plot(bpl_vals, util_vals,
                color=color_by_cluster[clusters],
                marker=marker_by_cores[cores],
                linewidth=2, markersize=7, label=label)

    ax.set_xlabel('Bytes / Lane', fontsize=FONT_SIZE, fontweight='bold')
    ax.set_ylabel('FPU Utilization (%)', fontsize=FONT_SIZE, fontweight='bold')
    ax.set_title(f'{display_benchmark_name(benchmark)}', fontsize=FONT_SIZE + 1, fontweight='bold')
    ax.set_xscale('log', base=2)
    ax.set_xticks(all_bpl_vals)
    ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))
    ax.tick_params(axis='both', labelsize=TICK_FONT_SIZE)
    ax.legend(frameon=True, fontsize=TICK_FONT_SIZE, loc="best")
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_ylim(0, 105)

    fig.suptitle('FPU Utilization vs Bytes/Lane — AraXL Configurations', fontsize=FONT_SIZE + 2, fontweight='bold')
    plt.tight_layout()

    logs_dir = os.path.join(APPS_LOGS_DIR, benchmark)
    output_file = os.path.join(logs_dir, f'{benchmark}_utilization.svg')
    plt.savefig(output_file, format='svg', bbox_inches='tight')
    print(f"Plot saved to {output_file}")
    plt.close(fig)

# Plot: combined multi-panel summary across all benchmarks
os.makedirs(PLOTS_DIR, exist_ok=True)
combined_output = os.path.join(PLOTS_DIR, 'benchmark_utilization_comparison.svg')
plot_combined_summary(all_data, combined_output, cluster_colors, core_markers, font_size=FONT_SIZE, tick_font_size=TICK_FONT_SIZE)

# --- _canneal: 16C total 32-bit ops / (sw-cycles * total lanes * 2) ratio ---
canneal_logs_dir = os.path.join(APPS_LOGS_DIR, '_canneal')
if os.path.isdir(canneal_logs_dir):
    reference_ops, reference_log = get_canneal_reference_total_ops(canneal_logs_dir)
    print(f"\n--- _canneal: 16C total 32-bit ops / (sw-cycles * total lanes * 2) ---")
    if reference_ops is not None:
        print(f"Reference total 32-bit ops: {reference_ops} from {reference_log}")
    print(f"{'Log file':<35} {'sw-cycles':>12} {'lanes':>8} {'total ops':>12} {'util (%)':>12}")
    print("-" * 80)
    utils = []
    for fname in sorted(os.listdir(canneal_logs_dir)):
        if not fname.endswith('.log'):
            continue
        parsed = parse_filename(fname)
        if parsed is None or parsed[0][2] is not None:
            continue
        config, _ = parsed
        lanes, clusters, cores = config
        total_nr_lanes = lanes * clusters * (cores if cores is not None else 1)
        sw, _ = parse_canneal_counters(os.path.join(canneal_logs_dir, fname))
        if sw is not None and reference_ops is not None:
            util_pct = (reference_ops / (sw * total_nr_lanes * 2.0)) * 100.0
            utils.append(util_pct)
            print(f"{fname:<35} {sw:>12} {total_nr_lanes:>8} {reference_ops:>12} {util_pct:>12.6f}")
        else:
            print(f"{fname:<35}  (missing data)")
    if utils:
        print("-" * 80)
        print(f"{'Overall average util (%):':<64} {sum(utils)/len(utils):>12.6f}")

# --- _spmv: avg FPU cycles / util from log ---
spmv_logs_dir = os.path.join(APPS_LOGS_DIR, '_spmv')
if os.path.isdir(spmv_logs_dir):
    print(f"\n--- _spmv: avg FPU cycles / util ---")
    print(f"{'Log file':<35} {'sw-cycles':>12} {'avg fpu-cycles':>16} {'util (%)':>12}")
    print("-" * 80)
    utils = []
    for fname in sorted(os.listdir(spmv_logs_dir)):
        if not fname.endswith('.log'):
            continue
        parsed = parse_filename(fname)
        if parsed is None or parsed[0][2] is not None:
            continue
        sw, avg_fpu, util_pct = parse_spmv_data(os.path.join(spmv_logs_dir, fname))
        if sw is not None:
            utils.append(util_pct)
            print(f"{fname:<35} {sw:>12} {avg_fpu:>16.2f} {util_pct:>12.6f}")
        else:
            print(f"{fname:<35}  (missing data)")
    if utils:
        print("-" * 80)
        print(f"{'Overall average util (%):':<64} {sum(utils)/len(utils):>12.6f}")
