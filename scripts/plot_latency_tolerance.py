#!/usr/bin/env python3
"""Plot benchmark utilization with and without interface latency.

The input layout is expected to be::

    logs/<benchmark>/<configuration>_<bytes>B_{0mem,4mem,1ring,1cva6}.log

For example, ``logs/fmatmul/4L_16C_64B_0mem.log`` can be paired with
``logs/fmatmul/4L_16C_64B_4mem.log``. Only exact baseline/latency pairs are
plotted.
"""

import argparse
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import PercentFormatter


# Shared qualitative palette, same order as plot_muticore.py's PALETTE and
# plot_benchmark.py's cluster_colors, so all three scripts look consistent.
PALETTE = [
    "#2A9D8F", "#E76F51", "#E9C46A", "#264653",
    "#F4A261", "#6A4C93", "#1982C4", "#8AC926",
]


def apply_font_style(font_size: int) -> None:
    """Apply plot text sizing, keeping the default (basic) system font."""
    plt.rcParams.update(
        {
            "font.size": font_size,
            "axes.titlesize": font_size + 3,
            "axes.titleweight": "semibold",
            "axes.labelsize": font_size + 2,
            "xtick.labelsize": font_size,
            "ytick.labelsize": font_size,
            "legend.fontsize": font_size,
            "figure.titlesize": font_size + 6,
        }
    )


BYTES_PER_LANE = (64, 128, 256, 512)
COMPARISON_VARIANTS = {
    "memory": "4mem",
    "ring": "1ring",
    "request": "1cva6",
}
COMPARISON_TITLES = {
    "memory": "GLSU",
    "ring": "RINGI",
    "request": "REQI",
}
LOG_NAME_RE = re.compile(
    r"^(?P<configuration>.*?)(?P<bytes>\d+)B_"
    r"(?P<variant>0mem|4mem|1ring|1cva6)\.log$"
)
UTIL_RE = re.compile(
    r"\butil\s*:\s*(?P<util>[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
    r"(?:[eE][-+]?\d+)?)\s*%?"
)


@dataclass(frozen=True, order=True)
class Series:
    benchmark: str
    configuration: str


def parse_args() -> argparse.Namespace:
    repository_root = Path(__file__).resolve().parents[1]
    default_logs = repository_root / "apps" / "logs"
    default_plots_dir = repository_root / "plots"

    parser = argparse.ArgumentParser(
        description="Plot benchmark latency tolerance from paired logs."
    )
    parser.add_argument(
        "--logs-dir",
        type=Path,
        default=default_logs,
        help=f"benchmark logs directory (default: {default_logs})",
    )
    parser.add_argument(
        "--comparison",
        choices=("memory", "ring", "request", "both", "all"),
        default="all",
        help=(
            "latency source to plot; 'both' selects memory and ring, while "
            "'all' also includes the request interface (default: all)"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "SVG output path (default: plots/memory_ring_request_latency_tolerance.svg "
            "for all)"
        ),
    )
    parser.add_argument(
        "--font-size",
        type=int,
        default=20,
        help="Base font size for plot text (default: 14)",
    )
    return parser.parse_args()


def read_last_utilization(log_file: Path) -> float | None:
    """Return the last ``util:`` value in a log, or None when absent."""
    utilization = None
    with log_file.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = UTIL_RE.search(line)
            if match:
                utilization = float(match.group("util"))
    return utilization


def discover_pairs(
    logs_dir: Path, comparison_variant: str = "4mem"
) -> tuple[dict[Series, dict[int, tuple[Path, Path]]], list[Path]]:
    """Find exact 0mem/comparison pairs by benchmark and configuration."""
    files_by_key: dict[tuple[str, str, int], dict[str, Path]] = defaultdict(dict)

    for benchmark_dir in sorted(logs_dir.iterdir()):
        if not benchmark_dir.is_dir():
            continue
        for log_file in sorted(benchmark_dir.glob("*.log")):
            match = LOG_NAME_RE.fullmatch(log_file.name)
            if not match:
                continue
            variant = match.group("variant")
            if variant not in ("0mem", comparison_variant):
                continue
            bytes_per_lane = int(match.group("bytes"))
            if bytes_per_lane not in BYTES_PER_LANE:
                continue
            key = (
                benchmark_dir.name,
                match.group("configuration").rstrip("_"),
                bytes_per_lane,
            )
            files_by_key[key][variant] = log_file

    pairs: dict[Series, dict[int, tuple[Path, Path]]] = defaultdict(dict)
    unpaired: list[Path] = []
    for (benchmark, configuration, bytes_per_lane), variant_files in files_by_key.items():
        if "0mem" not in variant_files or comparison_variant not in variant_files:
            unpaired.extend(variant_files.values())
            continue
        series = Series(benchmark, configuration)
        pairs[series][bytes_per_lane] = (
            variant_files["0mem"],
            variant_files[comparison_variant],
        )

    return dict(pairs), sorted(unpaired)


def load_utilizations(
    pairs: dict[Series, dict[int, tuple[Path, Path]]],
) -> dict[Series, dict[int, tuple[float, float]]]:
    """Read utilization from each pair and discard pairs with missing values."""
    data: dict[Series, dict[int, tuple[float, float]]] = defaultdict(dict)

    for series, points in sorted(pairs.items()):
        for bytes_per_lane, (zero_mem, one_mem) in sorted(points.items()):
            zero_util = read_last_utilization(zero_mem)
            one_util = read_last_utilization(one_mem)
            if zero_util is None or one_util is None:
                missing = [
                    str(path)
                    for path, value in ((zero_mem, zero_util), (one_mem, one_util))
                    if value is None
                ]
                print(
                    "warning: no 'util:' value in " + ", ".join(missing),
                    file=sys.stderr,
                )
                continue
            data[series][bytes_per_lane] = (zero_util, one_util)

    return dict(data)


def series_label(series: Series, configurations_per_benchmark: dict[str, int]) -> str:
    """Include the configuration only when a benchmark has more than one."""
    benchmark = series.benchmark.lstrip("_")
    if configurations_per_benchmark[series.benchmark] == 1:
        return benchmark
    configuration = series.configuration or "default"
    return f"{benchmark} ({configuration})"


def plot_latency_tolerance(
    comparison_data: dict[
        str, dict[Series, dict[int, tuple[float, float]]]
    ],
    output: Path,
) -> None:
    """Plot one side-by-side panel per latency comparison."""
    all_series = sorted(
        {series for data in comparison_data.values() for series in data}
    )
    configurations_per_benchmark: dict[str, int] = defaultdict(int)
    for series in all_series:
        configurations_per_benchmark[series.benchmark] += 1

    colors = [PALETTE[index % len(PALETTE)] for index in range(len(all_series))]
    color_by_series = {
        series: colors[index] for index, series in enumerate(all_series)
    }
    panel_count = len(comparison_data)
    fig, axes = plt.subplots(
        1,
        panel_count,
        figsize=(6 * panel_count, 7),
        sharex=True,
        sharey=True,
        squeeze=False,
    )
    annotation_offsets = (12, -16, 28)

    for panel_index, (comparison, data) in enumerate(comparison_data.items()):
        ax = axes[0, panel_index]
        for series_index, series in enumerate(all_series):
            if series not in data:
                continue
            points = data[series]
            x_values = sorted(points)
            baseline = [points[value][0] for value in x_values]
            with_latency = [points[value][1] for value in x_values]
            color = color_by_series[series]

            ax.plot(
                x_values,
                baseline,
                color=color,
                linestyle="-",
                marker="o",
                linewidth=2,
                markersize=5,
            )
            ax.plot(
                x_values,
                with_latency,
                color=color,
                linestyle=":",
                marker="o",
                linewidth=2,
                markersize=5,
            )

            x_value, (zero_util, comparison_util) = max(
                points.items(), key=lambda item: item[1][0] - item[1][1]
            )
            displayed_drop = max(zero_util - comparison_util, 0.0)
            ax.vlines(
                x_value,
                comparison_util,
                zero_util,
                color=color,
                linestyle="--",
                linewidth=2,
                zorder=4,
            )
            ax.scatter(
                [x_value],
                [comparison_util],
                color=color,
                edgecolor="black",
                marker="*",
                s=130,
                clip_on=False,
                zorder=5,
            )
            text_offset = (
                -10 if x_value >= 128 else 10,
                annotation_offsets[series_index % len(annotation_offsets)],
            )
            ax.annotate(
                f"{displayed_drop:.2f}%",
                xy=(x_value, comparison_util),
                xytext=text_offset,
                textcoords="offset points",
                horizontalalignment="right" if x_value >= 128 else "left",
                verticalalignment="bottom" if text_offset[1] >= 0 else "top",
                color=color,
                fontweight="bold",
                arrowprops={"arrowstyle": "-", "color": color, "linewidth": 1},
                bbox={
                    "boxstyle": "round,pad=0.2",
                    "facecolor": "white",
                    "edgecolor": color,
                    "alpha": 0.9,
                },
                zorder=6,
            )

        ax.set_xscale("log", base=2)
        ax.set_xlim(BYTES_PER_LANE[0], BYTES_PER_LANE[-1])
        ax.set_xticks(
            BYTES_PER_LANE, labels=[str(value) for value in BYTES_PER_LANE]
        )
        ax.set_ylim(0, 105)
        ax.yaxis.set_major_formatter(PercentFormatter(xmax=100))
        ax.set_xlabel("Bytes / Lane", fontweight="bold")
        ax.set_title(COMPARISON_TITLES[comparison])
        ax.grid(True, which="major", linestyle="--", alpha=0.3)

    axes[0, 0].set_ylabel("FPU Utilization (%)", fontweight="bold")
    # fig.suptitle("Benchmark latency tolerance")

    benchmark_handles = [
        Line2D(
            [0],
            [0],
            color=color_by_series[series],
            linewidth=2,
            marker="o",
            label=series_label(series, configurations_per_benchmark),
        )
        for series in all_series
    ]
    style_handles = [
        Line2D(
            [0],
            [0],
            color="black",
            linewidth=2,
            linestyle="-",
            label="baseline",
        )
    ] + [
        Line2D(
            [0],
            [0],
            color="black",
            linewidth=2,
            linestyle=":",
            label=COMPARISON_VARIANTS[comparison],
        )
        for comparison in comparison_data
    ]
    handles = benchmark_handles + style_handles
    fig.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.005),
        ncol=min(7, len(handles)),
    )
    fig.tight_layout(rect=(0, 0.14, 1, 0.94), w_pad=0.5)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, format="svg", bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    apply_font_style(args.font_size)
    logs_dir = args.logs_dir.resolve()
    if not logs_dir.is_dir():
        print(f"error: logs directory does not exist: {logs_dir}", file=sys.stderr)
        return 2

    output = args.output
    if output is None:
        default_plots_dir = Path(__file__).resolve().parents[1] / "plots"
        if args.comparison == "all":
            output = default_plots_dir / "memory_ring_request_latency_tolerance.svg"
        elif args.comparison == "both":
            output = default_plots_dir / "memory_ring_latency_tolerance.svg"
        else:
            output = default_plots_dir / f"{args.comparison}_latency_tolerance.svg"
    elif output.suffix.lower() != ".svg":
        print(f"error: output file must use the .svg extension: {output}", file=sys.stderr)
        return 2

    if args.comparison == "all":
        comparisons = ("memory", "ring", "request")
    elif args.comparison == "both":
        comparisons = ("memory", "ring")
    else:
        comparisons = (args.comparison,)
    comparison_data = {}
    for comparison in comparisons:
        comparison_variant = COMPARISON_VARIANTS[comparison]
        pairs, unpaired = discover_pairs(logs_dir, comparison_variant)
        unpaired_by_benchmark = Counter(path.parent.name for path in unpaired)
        for benchmark, count in sorted(unpaired_by_benchmark.items()):
            print(
                f"warning [{comparison}]: skipping {count} unpaired log(s) "
                f"in {benchmark}",
                file=sys.stderr,
            )

        data = load_utilizations(pairs)
        if not data:
            print(
                f"error: no paired 0mem/{comparison_variant} logs with "
                f"utilization found in {logs_dir}",
                file=sys.stderr,
            )
            return 1
        comparison_data[comparison] = data

    plot_latency_tolerance(comparison_data, output)
    point_count = sum(
        len(points)
        for data in comparison_data.values()
        for points in data.values()
    )
    print(
        f"Plotted {len(comparison_data)} comparison panel(s) "
        f"({point_count} paired points) to {output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
