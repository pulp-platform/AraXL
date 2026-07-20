#!/usr/bin/env python3
"""
Benchmark visualization script
Plots FPU utilization vs Bytes/Lane for different AraXL cluster configurations.
Log files are read from apps/logs*/<benchmark>/ with naming like:
{L}L_{C}C_{B}B_0mem.log or {L}L_{C}C_{core}core_{B}B_0mem.log
Only logs without an explicit multi-core suffix are plotted.
"""

import os
import re
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

APPS_LOGS_DIR = os.path.join(os.path.dirname(__file__), '..', 'apps', 'logs')


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
cluster_colors = ['#2A9D8F', '#E9C46A', '#F4A261', '#264653', '#E76F51', '#6A4C93', '#1982C4', '#8AC926']
core_markers = ['o', 's', '^', 'D', 'v', 'P', 'X', '*']

for benchmark, data in all_data.items():
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

    ax.set_xlabel('Bytes / Lane', fontsize=12, fontweight='bold')
    ax.set_ylabel('FPU Utilization (%)', fontsize=12, fontweight='bold')
    ax.set_title(f'{benchmark}', fontsize=13, fontweight='bold')
    ax.set_xscale('log', base=2)
    ax.xaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{int(v)}'))
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_ylim(0, 105)

    fig.suptitle('FPU Utilization vs Bytes/Lane — AraXL Configurations', fontsize=14, fontweight='bold')
    plt.tight_layout()

    logs_dir = os.path.join(APPS_LOGS_DIR, benchmark)
    output_file = os.path.join(logs_dir, f'{benchmark}_utilization.svg')
    plt.savefig(output_file, format='svg', bbox_inches='tight')
    print(f"Plot saved to {output_file}")
    plt.close(fig)

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
