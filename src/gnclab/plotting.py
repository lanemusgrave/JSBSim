"""Plot helpers: stacked time histories saved to ``outputs/<module>/``."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping, Sequence

import matplotlib.pyplot as plt
import pandas as pd

from gnclab.sim import OUTPUT_DIR


def outdir(module: str) -> Path:
    """``outputs/<module>/`` (created if needed)."""
    p = OUTPUT_DIR / module
    p.mkdir(parents=True, exist_ok=True)
    return p


def timehistory(
    data: pd.DataFrame | Mapping[str, pd.DataFrame],
    columns: Sequence[str],
    title: str = "",
    ylabels: Mapping[str, str] | None = None,
    sharex: bool = True,
    figsize=(9, None),
):
    """Stack one subplot per column.  ``data`` may be a dict of DataFrames to
    overlay several runs (the dict keys become the legend)."""
    runs = data if isinstance(data, Mapping) else {"": data}
    n = len(columns)
    fig, axes = plt.subplots(n, 1, sharex=sharex,
                             figsize=(figsize[0], figsize[1] or 1.8 * n + 1))
    if n == 1:
        axes = [axes]
    for ax, col in zip(axes, columns):
        for label, df in runs.items():
            if col in df:
                ax.plot(df.index, df[col], label=label or None)
        ax.set_ylabel((ylabels or {}).get(col, col))
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time [s]")
    if len(runs) > 1:
        axes[0].legend(loc="best", fontsize="small")
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    return fig, axes


def save(fig, module: str, name: str, show: bool = False) -> Path:
    """Save ``fig`` as ``outputs/<module>/<name>.png`` and optionally show it."""
    path = outdir(module) / f"{name}.png"
    fig.savefig(path, dpi=110)
    print(f"saved {path.relative_to(OUTPUT_DIR.parent)}")
    if show:
        plt.show()
    plt.close(fig)
    return path
