#!/usr/bin/env python3
"""Recreate the Lanes-vs-VLEN design-space scatter plot from
araxl_relatedworks_data.txt (see that file for the column format)."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, NullFormatter

COLORS = {
    "riscv": "#a83231",      # dark red, matches original RISC-V markers
    "non_riscv": "#12699f",  # blue, matches original Non-RISC-V markers
}
LABELS = {
    "riscv": "RISC-V",
    "non_riscv": "Non-RISC-V",
}
FONT_SIZE = 12


def read_data(path):
    points = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            name, lanes, vlen, category = line.split()
            points.append((name, int(lanes), int(vlen), category))
    return points


def plot(points, out_path):
    fig, ax = plt.subplots(figsize=(6.5, 4.5))

    # AraXL designs are marked with a star instead of the default circle.
    is_araxl = lambda name: "araxl" in name.lower()

    for category in ("riscv", "non_riscv"):
        others = [(l, v) for n, l, v, c in points if c == category and not is_araxl(n)]
        araxl = [(l, v) for n, l, v, c in points if c == category and is_araxl(n)]
        if others:
            xs, ys = zip(*others)
            ax.scatter(xs, ys, color=COLORS[category], label=LABELS[category], zorder=3)
        if araxl:
            xs, ys = zip(*araxl)
            ax.scatter(
                xs, ys, color=COLORS[category], marker="*", s=160,
                zorder=4,
            )

    for name, lanes, vlen, _ in points:
        ax.annotate(
            name,
            (lanes, vlen),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=FONT_SIZE,
        )

    ax.set_xscale("log", base=2)
    ax.set_yscale("log", base=2)
    ax.set_xlabel("Lanes (FPUs / instruction)", fontsize=FONT_SIZE, fontweight="bold")
    ax.set_ylabel("VLEN (bits)", fontsize=FONT_SIZE, fontweight="bold")

    # Tick at every value present in the data, labeled as a plain number
    # instead of a power-of-2 exponent, on both axes.
    xticks = sorted({lanes for _, lanes, _, _ in points})
    yticks = sorted({vlen for _, _, vlen, _ in points})
    ax.set_xticks(xticks)
    ax.set_yticks(yticks)
    ax.xaxis.set_major_formatter(FuncFormatter(lambda x, _pos: f"{int(x)}"))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda y, _pos: f"{int(y)}"))
    ax.xaxis.set_minor_formatter(NullFormatter())
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.grid(True, which="both", linestyle="-", linewidth=0.5, color="0.85")
    ax.set_axisbelow(True)
    ax.legend(frameon=True, fontsize=FONT_SIZE - 1, loc="upper left")

    fig.tight_layout()
    fig.savefig(out_path)
    print(f"Saved plot to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    plots_dir = Path(__file__).parent.parent / "plots"
    parser.add_argument(
        "--data",
        default=plots_dir / "araxl_relatedworks_data.txt",
        type=Path,
    )
    parser.add_argument(
        "--out",
        default=plots_dir / "araxl_relatedworks_plot.pdf",
        type=Path,
    )
    args = parser.parse_args()

    points = read_data(args.data)
    plot(points, args.out)


if __name__ == "__main__":
    main()
