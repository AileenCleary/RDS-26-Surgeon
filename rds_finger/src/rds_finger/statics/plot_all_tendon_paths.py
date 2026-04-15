"""
Generate tangent visualization plots for every tendon path defined in config.py.

For each tendon, saves:
  - One PNG per consecutive segment showing all tangent candidates vs the chosen
    tangent (2D, in the local pulley plane).
  - One PNG of the full tendon path projected onto the xy and xz planes.
  - One PNG of the full 3D path with all candidates (light dashed) and the chosen
    path (solid) overlaid.

Run from the repo root with the package on the Python path:

    cd rds_finger
    python -m rds_finger.statics.plot_all_tendon_paths

Output lands in  tendon_path_figures/  relative to the working directory.
"""

import matplotlib
matplotlib.use("Agg")  # non-interactive backend — must come before pyplot import

import re
from pathlib import Path

import matplotlib.pyplot as plt

from rds_finger.config import TENDON_PATH
from rds_finger.types.fixed_points import Point3D
from rds_finger.types.pulleys import Pulley
from rds_finger.statics.plot_tangents import (
    save_point_pulley_all_vs_chosen,
    save_pulley_pulley_all_vs_chosen,
    plot_tendon_path_3d,
    plot_tendon_path_2d_projections,
)

OUTPUT_ROOT = Path("tendon_path_figures")


# ---------------------------------------------------------------------------
# Label helpers
# ---------------------------------------------------------------------------

def slugify(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    value = re.sub(r"_+", "_", value)
    return value.strip("_") or "item"


def element_label(obj) -> str:
    if isinstance(obj, Pulley):
        tendon = getattr(obj, "tendon", "tendon")
        shaft  = getattr(obj, "shaft",  "shaft")
        name   = getattr(obj, "name",   None) or "pulley"
        return f"pulley_{tendon}_{shaft}_{name}"
    point_type = getattr(obj, "type",   "POINT")
    tendon     = getattr(obj, "tendon", "tendon")
    return f"point_{tendon}_{point_type}"


def segment_label(a, b, idx: int) -> str:
    a_kind = "point" if isinstance(a, Point3D) else "pulley"
    b_kind = "point" if isinstance(b, Point3D) else "pulley"
    return (
        f"{idx:02d}_{a_kind}_to_{b_kind}"
        f"_{slugify(element_label(a))}_to_{slugify(element_label(b))}"
    )


# ---------------------------------------------------------------------------
# Per-segment 2-D plots
# ---------------------------------------------------------------------------

def save_2d_segments_for_tendon(
        tendon_name: str,
        path: list,
        pairwise_dir: Path,
) -> list:
    """Save one all-vs-chosen 2-D PNG per consecutive segment of *path*."""
    saved = []
    for idx, (a, b) in enumerate(zip(path, path[1:])):
        label = segment_label(a, b, idx)
        out   = pairwise_dir / f"{label}_all_vs_chosen.png"

        if isinstance(a, Point3D) and isinstance(b, Pulley):
            save_point_pulley_all_vs_chosen(a, b, str(out))

        elif isinstance(a, Pulley) and isinstance(b, Point3D):
            # Tangent geometry is symmetric: pass (point, pulley) regardless of
            # travel direction; the chosen tangent is still selected by pulley.dir.
            save_point_pulley_all_vs_chosen(b, a, str(out))

        elif isinstance(a, Pulley) and isinstance(b, Pulley):
            save_pulley_pulley_all_vs_chosen(a, b, str(out))

        else:
            # Point3D -> Point3D: no tangent to compute — skip.
            print(f"  [skip] {tendon_name} seg {idx}: point->point segment")
            continue

        saved.append(out)

    return saved


# ---------------------------------------------------------------------------
# Full-path 2-D projection plots (xy and xz)
# ---------------------------------------------------------------------------

def save_2d_projections_for_tendon(
        tendon_name: str,
        path: list,
        combined_dir: Path,
) -> Path:
    """Save a side-by-side xy / xz projection PNG for one tendon."""
    fig, _, _ = plot_tendon_path_2d_projections(tendon_name, path)
    out = combined_dir / f"{slugify(tendon_name)}_path_2d_projections.png"
    fig.savefig(str(out), dpi=220, bbox_inches="tight")
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# Full-path 3-D plot
# ---------------------------------------------------------------------------

def save_3d_path_for_tendon(
        tendon_name: str,
        path: list,
        combined_dir: Path,
) -> Path:
    """Save the 3-D full-path visualisation for one tendon."""
    fig, ax = plot_tendon_path_3d(tendon_name, path)
    out = combined_dir / f"{slugify(tendon_name)}_path_3d.png"
    fig.savefig(str(out), dpi=220, bbox_inches="tight")
    plt.close(fig)
    return out


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main(
        tendon_paths: dict | None = None,
        output_root: Path | str = OUTPUT_ROOT,
) -> None:
    if tendon_paths is None:
        tendon_paths = TENDON_PATH

    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    for tendon_name, path in tendon_paths.items():
        pairwise_dir = output_root / slugify(tendon_name) / "pairwise_segments"
        combined_dir = output_root / slugify(tendon_name) / "combined_route"
        pairwise_dir.mkdir(parents=True, exist_ok=True)
        combined_dir.mkdir(parents=True, exist_ok=True)

        print(f"[{tendon_name}] generating 2D segment plots …")
        seg_files = save_2d_segments_for_tendon(tendon_name, path, pairwise_dir)
        print(f"  {len(seg_files)} segment PNG(s) -> {pairwise_dir}")

        print(f"[{tendon_name}] generating 2D projection plots …")
        proj_file = save_2d_projections_for_tendon(tendon_name, path, combined_dir)
        print(f"  2D projection PNG -> {proj_file}")

        print(f"[{tendon_name}] generating 3D path plot …")
        path_file = save_3d_path_for_tendon(tendon_name, path, combined_dir)
        print(f"  3D path PNG -> {path_file}")


if __name__ == "__main__":
    main()
