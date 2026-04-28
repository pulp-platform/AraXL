#!/usr/bin/env python3
"""
Benchmark visualization script
Plots FPU utilization vs Bytes/Lane for different AraXL cluster configurations.
Log files are read from apps/logs/<benchmark>/ with naming: {L}L_{C}C_{B}B_0mem.log
"""

import os
import re
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

APPS_LOGS_DIR = os.path.join(os.path.dirname(__file__), '..', 'apps', 'logs')
BENCHMARKS = ['fmatmul','fconv2d','fdotproduct','exp', 'softmax','_axpy','_blackscholes','_pathfinder','_swaptions','_lavaMD','_streamcluster']

def parse_log_utilization(filepath):
    """Extract utilization percentage from last ~30 lines of a log file."""
    util = None
    with open(filepath, 'r') as f:
        lines = f.readlines()
    for line in reversed(lines[-60:]):
        m = re.search(r'util:([\d.]+)%', line)
        if m:
            util = float(m.group(1))
            break
    return util

def parse_canneal_fpu_ratio(filepath):
    """Return (sw_cycles, avg_fpu_cycles, ratio) for a _canneal log file."""
    sw_cycles = None
    fpu_cycles_list = []
    with open(filepath, 'r') as f:
        for line in f:
            m = re.search(r'\[sw-cycles\][=:]\s*(\d+)', line)
            if m:
                sw_cycles = int(m.group(1))
            m2 = re.search(r'\[fpu-cycles\]\s*:\s*(\d+)', line)
            if m2:
                fpu_cycles_list.append(int(m2.group(1)))
    if sw_cycles is not None and fpu_cycles_list:
        avg_fpu = sum(fpu_cycles_list) / len(fpu_cycles_list)
        return sw_cycles, avg_fpu, avg_fpu / sw_cycles
    return None, None, None

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
    """Parse {L}L_{C}C_{B}B_0mem.log -> (lanes, clusters, bytes_per_lane)."""
    m = re.match(r'(\d+)L_(\d+)C_(\d+)B_\d+mem\.log', filename)
    if m:
        return int(m.group(1)), int(m.group(2)), int(m.group(3))
    return None

# Collect data per benchmark: {benchmark: {clusters: {bytes: util}}}
all_data = {}

for benchmark in BENCHMARKS:
    logs_dir = os.path.join(APPS_LOGS_DIR, benchmark)
    if not os.path.isdir(logs_dir):
        print(f"WARNING: directory not found: {logs_dir}")
        continue
    data = defaultdict(dict)
    for fname in os.listdir(logs_dir):
        parsed = parse_filename(fname)
        if parsed is None:
            continue
        lanes, clusters, bytes_per_lane = parsed
        fpath = os.path.join(logs_dir, fname)
        util = parse_log_utilization(fpath)
        if util is not None:
            data[clusters][bytes_per_lane] = util
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
    for clusters in sorted(data):
        for bpl in sorted(data[clusters]):
            print(f"4L_{clusters}C      {bpl:<12} {data[clusters][bpl]:.3f}%")

# Plot: one figure per benchmark, saved as SVG in the respective logs folder
colors = ['#2A9D8F', '#E9C46A', '#F4A261', '#264653', '#E76F51']
markers = ['o', 's', '^', 'D', 'v']

for benchmark, data in all_data.items():
    fig, ax = plt.subplots(figsize=(8, 6))

    for i, clusters in enumerate(sorted(data)):
        bpl_vals = sorted(data[clusters])
        util_vals = [data[clusters][b] for b in bpl_vals]
        label = f'4L_{clusters}C'
        ax.plot(bpl_vals, util_vals,
                color=colors[i % len(colors)],
                marker=markers[i % len(markers)],
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

# --- _canneal: avg FPU cycles / sw-cycles ratio ---
canneal_logs_dir = os.path.join(APPS_LOGS_DIR, '_canneal')
if os.path.isdir(canneal_logs_dir):
    print(f"\n--- _canneal: avg FPU cycles / sw-cycles ---")
    print(f"{'Log file':<35} {'sw-cycles':>12} {'avg fpu-cycles':>16} {'util (%)':>12}")
    print("-" * 80)
    ratios = []
    for fname in sorted(os.listdir(canneal_logs_dir)):
        if not fname.endswith('.log'):
            continue
        sw, avg_fpu, ratio = parse_canneal_fpu_ratio(os.path.join(canneal_logs_dir, fname))
        if ratio is not None:
            ratios.append(ratio)
            print(f"{fname:<35} {sw:>12} {avg_fpu:>16.2f} {ratio*100:>12.6f}")
        else:
            print(f"{fname:<35}  (missing data)")
    if ratios:
        print("-" * 80)
        print(f"{'Overall average util (%):':<64} {sum(ratios)/len(ratios)*100:>12.6f}")

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
        sw, avg_fpu, util_pct = parse_spmv_data(os.path.join(spmv_logs_dir, fname))
        if sw is not None:
            utils.append(util_pct)
            print(f"{fname:<35} {sw:>12} {avg_fpu:>16.2f} {util_pct:>12.6f}")
        else:
            print(f"{fname:<35}  (missing data)")
    if utils:
        print("-" * 80)
        print(f"{'Overall average util (%):':<64} {sum(utils)/len(utils):>12.6f}")
