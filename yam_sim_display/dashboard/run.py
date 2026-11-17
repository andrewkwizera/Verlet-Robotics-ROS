#!/usr/bin/env python3
"""Installed entry point for the YAM operator dashboard.

Resolves its Python package and web assets from the ROS package share
directory, so it behaves identically from a ``colcon --symlink-install``
install space, a merged install space, and the source tree.
"""

from __future__ import annotations

import os
import sys


def _package_share() -> str:
    """Absolute path to the installed (or source) package share root."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return get_package_share_directory("yam_sim_display")
    except Exception:  # noqa: BLE001 - fall back to the source tree
        # <repo>/yam_sim_display/dashboard/run.py -> <repo>/yam_sim_display
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main() -> int:
    share = _package_share()
    dashboard_root = os.path.join(share, "dashboard")
    web_dir = os.path.join(dashboard_root, "web")

    if not os.path.isdir(web_dir):
        sys.stderr.write(
            "yam_dashboard: web assets not found at "
            f"{web_dir}. Is the package installed correctly?\n"
        )
        return 2

    # ``share`` is the parent of the ``dashboard`` package directory.
    if share not in sys.path:
        sys.path.insert(0, share)

    from dashboard.node import run  # noqa: PLC0415 - deferred until path is set

    return run(web_dir)


if __name__ == "__main__":
    sys.exit(main())
