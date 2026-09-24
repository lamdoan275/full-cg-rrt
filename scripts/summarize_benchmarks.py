#!/usr/bin/env python3
"""Aggregate benchmark_results/*/results.txt into one comparison table.

Each row is one run directory: mean +- sd over its successful trials, plus the
failure count. Sweep points labelled by run_param_sweep.sh also show the
parameters they were measured at.

Usage:
  python3 scripts/summarize_benchmarks.py [filter ...]

A filter is a substring matched against the directory name, e.g. "maze_20".
"""

import os
import re
import statistics as st
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "benchmark_results")

# label -> regex capturing one float per trial block
METRICS = [
    ("time s", r"time\s+([\d.]+) s"),
    ("cost m", r"cost\s+([\d.]+) m"),
    ("nodes", r"nodes\s+(\d+)"),
    ("replan", r"replan_count\s+(\d+)"),
    ("robot curv", r"robot mean/max\s+([\d.]+)"),
    ("rmse m", r"rmse\s+([\d.]+)"),
]


def read_run(path):
    results = os.path.join(path, "results.txt")
    if not os.path.isfile(results):
        return None
    text = open(results).read()

    blocks = re.split(r"\n(?=\d+,\s+TONG QUAN)", text)
    trials = []
    for block in blocks:
        if "TONG QUAN" not in block:
            continue
        row = {}
        for label, pattern in METRICS:
            m = re.search(pattern, block)
            if m:
                row[label] = float(m.group(1))
        if len(row) == len(METRICS):
            trials.append(row)

    params = {}
    pfile = os.path.join(path, "params.txt")
    if os.path.isfile(pfile):
        for line in open(pfile):
            if "=" in line:
                k, v = line.strip().split("=", 1)
                params[k] = v

    return {
        "name": os.path.basename(path.rstrip("/")),
        "trials": trials,
        "failed": len(re.findall(r"FAILED", text)),
        "params": params,
    }


def cell(values):
    if not values:
        return "-"
    mean = st.mean(values)
    sd = st.stdev(values) if len(values) > 1 else 0.0
    fmt = "%.0f+-%.0f" if mean >= 1000 else "%.3f+-%.3f"
    return fmt % (mean, sd)


def main():
    filters = sys.argv[1:]
    runs = []
    if not os.path.isdir(ROOT):
        print("no benchmark_results directory", file=sys.stderr)
        return 1
    for entry in sorted(os.listdir(ROOT)):
        path = os.path.join(ROOT, entry)
        if not os.path.isdir(path):
            continue
        if filters and not any(f in entry for f in filters):
            continue
        run = read_run(path)
        if run and run["trials"]:
            runs.append(run)

    if not runs:
        print("nothing to summarise")
        return 0

    head = ["run", "d", "k", "n", "fail"] + [m[0] for m in METRICS]
    rows = []
    for run in runs:
        p = run["params"]
        rows.append([
            run["name"],
            p.get("sample_max_d", "?"),
            p.get("number_node_k", "?"),
            str(len(run["trials"])),
            str(run["failed"]),
        ] + [cell([t[m[0]] for t in run["trials"]]) for m in METRICS])

    widths = [max(len(r[i]) for r in [head] + rows) for i in range(len(head))]
    line = "  ".join(h.ljust(w) for h, w in zip(head, widths))
    print(line)
    print("-" * len(line))
    for row in rows:
        print("  ".join(c.ljust(w) for c, w in zip(row, widths)))

    print()
    print("d = sample_max_d (cells, 0.05 m each), k = number_node_k, n = successful trials")
    print("robot curv is measured on the driven trajectory, not on the plan")
    return 0


if __name__ == "__main__":
    sys.exit(main())
