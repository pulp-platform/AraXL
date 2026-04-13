#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# SPDX-License-Identifier: Apache-2.0
#
# Convert MFPU, VSTU, and VLDU CSV trace files (generated with +define+PERFETTO_TRACE)
# into a Perfetto-compatible Chrome Trace Event Format JSON file.
#
# Usage:
#   python3 mfpu_trace_to_perfetto.py [--input-dir DIR] [--output FILE]
#
# The script reads all trace_mfpu_c*_l*.csv, trace_vstu_c*.csv, and
# trace_vldu_c*.csv files from the input directory and emits one combined trace viewable at
# https://ui.perfetto.dev

import argparse
import csv
import glob
import hashlib
import json
import os
import sys


TRACE_COLOR_PALETTE = [
    "thread_state_running",
    "thread_state_runnable",
    "thread_state_uninterruptible",
    "thread_state_iowait",
    "thread_state_unknown",
    "background_memory_dump",
    "light_memory_dump",
    "detailed_memory_dump",
    "good",
    "bad",
    "terrible",
    "yellow",
    "olive",
    "rail_response",
    "rail_animation",
    "rail_idle",
    "rail_load",
    "startup",
    "heap_dump_stack_frame",
    "heap_dump_object_type",
]


MFPU_TID_STRIDE = 10
VSTU_TID_BASE = 1000
VLDU_TID_BASE = 2000


def section_pid(cluster, unit, lane=None):
    """Return a stable process id for a viewer section."""
    cluster_base = cluster * 100
    if unit == "mfpu":
        return cluster_base + int(lane)
    if unit == "vstu":
        return cluster_base + 90
    if unit == "vldu":
        return cluster_base + 91
    raise ValueError(f"Unsupported unit for section pid: {unit}")


def instruction_color(ev):
    """Return a stable Perfetto/Chrome trace color name for an instruction."""
    color_key = ":".join([
        ev.get("unit", "unknown"),
        str(ev["cluster"]),
        str(ev.get("lane", -1)),
        str(ev["insn_id"]),
        ev["op"],
        str(ev["vl"]),
        str(ev["vd"]),
    ])
    color_hash = hashlib.sha256(color_key.encode("ascii")).digest()
    color_index = int.from_bytes(color_hash[:4], "big") % len(TRACE_COLOR_PALETTE)
    return TRACE_COLOR_PALETTE[color_index]


def parse_mfpu_trace_files(input_dir):
    """Parse all trace_mfpu_c*_l*.csv files and return raw MFPU events."""
    pattern = os.path.join(input_dir, "trace_mfpu_c*_l*.csv")
    files = sorted(glob.glob(pattern))
    skipped = 0
    events = []

    for fpath in files:
        with open(fpath, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ev = {
                        "unit":      "mfpu",
                        "phase":     row["phase"].strip(),
                        "timestamp": int(row["timestamp"].strip()),
                        "cluster":   int(row["cluster"].strip()),
                        "lane":      int(row["lane"].strip()),
                        "insn_id":   int(row["insn_id"].strip()),
                        "op":        row["op"].strip(),
                        "vl":        int(row["vl"].strip()),
                        "vd":        int(row["vd"].strip()),
                    }
                except (TypeError, ValueError, KeyError, AttributeError):
                    skipped += 1
                    continue
                vrf_addr = (row.get("vrf_addr") or "").strip()
                data = (row.get("data") or "").strip()
                be = (row.get("be") or "").strip()
                if vrf_addr:
                    ev["vrf_addr"] = vrf_addr
                if data:
                    ev["data"] = data
                if be:
                    ev["be"] = be
                events.append(ev)

    return files, events, skipped


def parse_vstu_trace_files(input_dir):
    """Parse all trace_vstu_c*.csv files and return raw VSTU events."""
    pattern = os.path.join(input_dir, "trace_vstu_c*.csv")
    files = sorted(glob.glob(pattern))
    skipped = 0
    events = []

    for fpath in files:
        with open(fpath, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ev = {
                        "unit":      "vstu",
                        "phase":     row["phase"].strip(),
                        "timestamp": int(row["timestamp"].strip()),
                        "cluster":   int(row["cluster"].strip()),
                        "insn_id":   int(row["insn_id"].strip()),
                        "op":        row["op"].strip(),
                        "vl":        int(row["vl"].strip()),
                        "vd":        int(row["vd"].strip()),
                    }
                except (TypeError, ValueError, KeyError, AttributeError):
                    skipped += 1
                    continue
                data = (row.get("data") or "").strip()
                strb = (row.get("strb") or "").strip()
                last = (row.get("last") or "").strip()
                if data:
                    ev["data"] = data
                if strb:
                    ev["strb"] = strb
                if last:
                    ev["last"] = last
                events.append(ev)

    return files, events, skipped


def parse_vldu_trace_files(input_dir):
    """Parse all trace_vldu_c*.csv files and return raw VLDU events."""
    pattern = os.path.join(input_dir, "trace_vldu_c*.csv")
    files = sorted(glob.glob(pattern))
    skipped = 0
    events = []

    for fpath in files:
        with open(fpath, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    ev = {
                        "unit":      "vldu",
                        "phase":     row["phase"].strip(),
                        "timestamp": int(row["timestamp"].strip()),
                        "cluster":   int(row["cluster"].strip()),
                        "insn_id":   int(row["insn_id"].strip()),
                        "op":        row["op"].strip(),
                        "vl":        int(row["vl"].strip()),
                        "vd":        int(row["vd"].strip()),
                    }
                except (TypeError, ValueError, KeyError, AttributeError):
                    skipped += 1
                    continue
                data = (row.get("data") or "").strip()
                resp = (row.get("resp") or "").strip()
                last = (row.get("last") or "").strip()
                if data:
                    ev["data"] = data
                if resp:
                    ev["resp"] = resp
                if last:
                    ev["last"] = last
                events.append(ev)

    return files, events, skipped


def parse_trace_files(input_dir):
    """Parse all supported trace CSV files and return a combined event list."""
    mfpu_files, mfpu_events, mfpu_skipped = parse_mfpu_trace_files(input_dir)
    vstu_files, vstu_events, vstu_skipped = parse_vstu_trace_files(input_dir)
    vldu_files, vldu_events, vldu_skipped = parse_vldu_trace_files(input_dir)

    if not mfpu_files and not vstu_files and not vldu_files:
        print(
            f"No trace files found matching {os.path.join(input_dir, 'trace_mfpu_c*_l*.csv')} "
            f"or {os.path.join(input_dir, 'trace_vstu_c*.csv')} "
            f"or {os.path.join(input_dir, 'trace_vldu_c*.csv')}",
            file=sys.stderr,
        )
        sys.exit(1)

    skipped = mfpu_skipped + vstu_skipped + vldu_skipped
    if skipped:
        print(f"Warning: skipped {skipped} malformed CSV row(s)", file=sys.stderr)

    return mfpu_events + vstu_events + vldu_events


def filter_and_scale_events(events):
    """Drop timestamp outliers and scale timestamps to match Perfetto units."""
    if not events:
        return events

    all_ts = sorted(e["timestamp"] for e in events)
    n = len(all_ts)
    if n > 4:
        q1 = all_ts[n // 4]
        q3 = all_ts[3 * n // 4]
        iqr = q3 - q1
        ts_lo = q1 - 3 * iqr
        ts_hi = q3 + 3 * iqr
        before = len(events)
        events = [e for e in events if ts_lo <= e["timestamp"] <= ts_hi]
        dropped = before - len(events)
        if dropped:
            print(f"Filtered {dropped} event(s) with outlier timestamps", file=sys.stderr)

    for event in events:
        event["timestamp"] = event["timestamp"] * 1000

    return events


def build_mfpu_trace(events):
    """Build MFPU Chrome Trace Event Format entries."""
    event_dur = 1000000
    open_events = {}
    trace_events = []

    for ev in events:
        key = (ev["cluster"], ev["lane"], ev["insn_id"])

        if ev["phase"] == "B":
            open_events[key] = ev
            color = instruction_color(ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "mfpu_begin",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "mfpu", ev["lane"]),
                "tid": 0,
                "args": {
                    "insn_id": ev["insn_id"],
                    "vl": ev["vl"],
                    "vd": ev["vd"],
                    "op": ev["op"],
                },
            })
        elif ev["phase"] == "W":
            args = {
                "insn_id": ev["insn_id"],
                "vl": ev["vl"],
                "vd": ev["vd"],
            }
            if "vrf_addr" in ev:
                args["vrf_addr"] = ev["vrf_addr"]
            if "data" in ev:
                args["data"] = ev["data"]
            if "be" in ev:
                args["be"] = ev["be"]
            color = instruction_color(ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "vrf_writeback",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "mfpu", ev["lane"]),
                "tid": 2,
                "args": args,
            })
        elif ev["phase"] == "E":
            begin = open_events.pop(key, None)
            duration = ev["timestamp"] - begin["timestamp"] if begin else 0
            color = instruction_color(begin if begin else ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "mfpu_end",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "mfpu", ev["lane"]),
                "tid": 1,
                "args": {
                    "insn_id": ev["insn_id"],
                    "vl": ev["vl"],
                    "vd": ev["vd"],
                    "duration": duration,
                },
            })

    return trace_events


def build_vstu_trace(events):
    """Build VSTU Chrome Trace Event Format entries."""
    event_dur = 1000000
    open_events = {}
    trace_events = []

    for ev in events:
        key = (ev["cluster"], ev["insn_id"])

        if ev["phase"] == "B":
            open_events[key] = ev
            color = instruction_color(ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "vstu_begin",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "vstu"),
                "tid": 0,
                "args": {
                    "insn_id": ev["insn_id"],
                    "vl": ev["vl"],
                    "vd": ev["vd"],
                    "op": ev["op"],
                },
            })
        elif ev["phase"] == "W":
            args = {
                "insn_id": ev["insn_id"],
                "vl": ev["vl"],
                "vd": ev["vd"],
            }
            if "data" in ev:
                args["data"] = ev["data"]
            if "strb" in ev:
                args["strb"] = ev["strb"]
            if "last" in ev:
                args["last"] = ev["last"]
            color = instruction_color(ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "axi_write",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "vstu"),
                "tid": 2,
                "args": args,
            })
        elif ev["phase"] == "E":
            begin = open_events.pop(key, None)
            duration = ev["timestamp"] - begin["timestamp"] if begin else 0
            color = instruction_color(begin if begin else ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "vstu_end",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "vstu"),
                "tid": 1,
                "args": {
                    "insn_id": ev["insn_id"],
                    "vl": ev["vl"],
                    "vd": ev["vd"],
                    "duration": duration,
                },
            })

    return trace_events


def build_vldu_trace(events):
    """Build VLDU Chrome Trace Event Format entries."""
    event_dur = 1000000
    open_events = {}
    trace_events = []

    for ev in events:
        key = (ev["cluster"], ev["insn_id"])

        if ev["phase"] == "B":
            open_events[key] = ev
            color = instruction_color(ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "vldu_begin",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "vldu"),
                "tid": 0,
                "args": {
                    "insn_id": ev["insn_id"],
                    "vl": ev["vl"],
                    "vd": ev["vd"],
                    "op": ev["op"],
                },
            })
        elif ev["phase"] == "R":
            args = {
                "insn_id": ev["insn_id"],
                "vl": ev["vl"],
                "vd": ev["vd"],
            }
            if "data" in ev:
                args["data"] = ev["data"]
            if "resp" in ev:
                args["resp"] = ev["resp"]
            if "last" in ev:
                args["last"] = ev["last"]
            color = instruction_color(ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "axi_read",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "vldu"),
                "tid": 2,
                "args": args,
            })
        elif ev["phase"] == "E":
            begin = open_events.pop(key, None)
            duration = ev["timestamp"] - begin["timestamp"] if begin else 0
            color = instruction_color(begin if begin else ev)
            trace_events.append({
                "name": ev["op"],
                "cat": "vldu_end",
                "cname": color,
                "ph": "X",
                "ts": ev["timestamp"],
                "dur": event_dur,
                "pid": section_pid(ev["cluster"], "vldu"),
                "tid": 1,
                "args": {
                    "insn_id": ev["insn_id"],
                    "vl": ev["vl"],
                    "vd": ev["vd"],
                    "duration": duration,
                },
            })

    return trace_events


def build_perfetto_trace(events):
    """Build one combined Perfetto trace for all supported trace sources."""
    events = filter_and_scale_events(events)
    events.sort(key=lambda event: event["timestamp"])

    mfpu_events = [event for event in events if event["unit"] == "mfpu"]
    vstu_events = [event for event in events if event["unit"] == "vstu"]
    vldu_events = [event for event in events if event["unit"] == "vldu"]

    trace_events = (
        build_mfpu_trace(mfpu_events)
        + build_vstu_trace(vstu_events)
        + build_vldu_trace(vldu_events)
    )

    metadata = []
    clusters = sorted({event["cluster"] for event in events})

    for cluster in clusters:
        cluster_mfpu_events = [event for event in mfpu_events if event["cluster"] == cluster]
        for lane in sorted({event["lane"] for event in cluster_mfpu_events}):
            lane_pid = section_pid(cluster, "mfpu", lane)
            metadata.extend([
                {
                    "name": "process_name",
                    "ph": "M",
                    "pid": lane_pid,
                    "tid": 0,
                    "args": {"name": f"Cluster {cluster} Lane {lane}"},
                },
                {
                    "name": "process_sort_index",
                    "ph": "M",
                    "pid": lane_pid,
                    "tid": 0,
                    "args": {"sort_index": lane},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": lane_pid,
                    "tid": 0,
                    "args": {"name": "Begin"},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": lane_pid,
                    "tid": 1,
                    "args": {"name": "End"},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": lane_pid,
                    "tid": 2,
                    "args": {"name": "VRF WB"},
                },
            ])

        if any(event["cluster"] == cluster for event in vldu_events):
            vldu_pid = section_pid(cluster, "vldu")
            metadata.extend([
                {
                    "name": "process_name",
                    "ph": "M",
                    "pid": vldu_pid,
                    "tid": 0,
                    "args": {"name": f"Cluster {cluster} VLDU"},
                },
                {
                    "name": "process_sort_index",
                    "ph": "M",
                    "pid": vldu_pid,
                    "tid": 0,
                    "args": {"sort_index": 90},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": vldu_pid,
                    "tid": 0,
                    "args": {"name": "Load Begin"},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": vldu_pid,
                    "tid": 1,
                    "args": {"name": "Load End"},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": vldu_pid,
                    "tid": 2,
                    "args": {"name": "AXI R Beat"},
                },
            ])

        if any(event["cluster"] == cluster for event in vstu_events):
            vstu_pid = section_pid(cluster, "vstu")
            metadata.extend([
                {
                    "name": "process_name",
                    "ph": "M",
                    "pid": vstu_pid,
                    "tid": 0,
                    "args": {"name": f"Cluster {cluster} VSTU"},
                },
                {
                    "name": "process_sort_index",
                    "ph": "M",
                    "pid": vstu_pid,
                    "tid": 0,
                    "args": {"sort_index": 91},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": vstu_pid,
                    "tid": 0,
                    "args": {"name": "Store Begin"},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": vstu_pid,
                    "tid": 1,
                    "args": {"name": "Store End"},
                },
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": vstu_pid,
                    "tid": 2,
                    "args": {"name": "AXI W Beat"},
                },
            ])

    return metadata + trace_events


def main():
    parser = argparse.ArgumentParser(
        description="Convert MFPU, VSTU, and VLDU CSV traces to one Perfetto JSON trace"
    )
    parser.add_argument(
        "--input-dir", "-i",
        default=".",
        help="Directory containing trace_mfpu_c*_l*.csv, trace_vstu_c*.csv, and trace_vldu_c*.csv files (default: current dir)",
    )
    parser.add_argument(
        "--output", "-o",
        default="perfetto_trace.json",
        help="Output Perfetto JSON trace file (default: perfetto_trace.json)",
    )
    args = parser.parse_args()

    events = parse_trace_files(args.input_dir)
    print(f"Parsed {len(events)} raw events")

    trace = build_perfetto_trace(events)
    print(f"Generated {len(trace)} Perfetto trace events")

    with open(args.output, "w") as f:
        json.dump({"traceEvents": trace, "displayTimeUnit": "ns"}, f, indent=1)

    print(f"Wrote {args.output} — open at https://ui.perfetto.dev")


if __name__ == "__main__":
    main()
