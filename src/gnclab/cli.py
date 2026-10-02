"""Common command-line flags for every lesson script.

Every exercise/solution script accepts:

``--show``  open interactive plot windows (default: just save PNGs to outputs/)
``--fast``  shorten the run (used by the automated smoke tests)
"""

from __future__ import annotations

import argparse


def parse_args(description: str = "", argv=None, extra=None) -> argparse.Namespace:
    """Parse the standard flags and pick a matplotlib backend.

    ``extra`` is an optional callable ``extra(parser)`` to add script-specific flags.
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--show", action="store_true", help="show plot windows")
    parser.add_argument("--fast", action="store_true", help="short run for smoke tests")
    if extra is not None:
        extra(parser)
    args = parser.parse_args(argv)
    import matplotlib

    if not args.show:
        matplotlib.use("Agg")  # file-only backend: works headless and in CI
    return args
