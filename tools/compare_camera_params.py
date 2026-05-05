#!/usr/bin/env python3
"""
Inspect and compare Picamera calibration .npz files (mtx, dist, optional H).

Usage:
  Scan a directory (default: <repo>/config) and report average px/mm from H for each file:
    python tools/compare_camera_params.py
    python tools/compare_camera_params.py --dir /path/to/npz

  Compare two calibration files (lens intrinsics + distortion + homography):
    python tools/compare_camera_params.py config/camera_a.npz config/camera_b.npz

Keys expected (match calibrate_picam): mtx, dist; optional H, rms.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

# Default frame used when measuring local px/mm (matches typical mapping; override with --width/--height)
_DEFAULT_WIDTH = 1280
_DEFAULT_HEIGHT = 720


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def load_npz(path: Path) -> dict[str, np.ndarray]:
    data = np.load(path, allow_pickle=False)
    return {k: data[k] for k in data.files}


def _fro_relative(a: np.ndarray, b: np.ndarray, eps: float = 1e-12) -> float:
    d = np.linalg.norm(a.astype(np.float64).ravel() - b.astype(np.float64).ravel())
    denom = np.linalg.norm(a.astype(np.float64).ravel()) + eps
    return float(d / denom)


def _pad_dist(d: np.ndarray, n: int) -> np.ndarray:
    d = np.asarray(d, dtype=np.float64).ravel()
    if d.size >= n:
        return d[:n]
    out = np.zeros(n, dtype=np.float64)
    out[: d.size] = d
    return out


def compare_intrinsics(
    mtx1: np.ndarray,
    dist1: np.ndarray,
    mtx2: np.ndarray,
    dist2: np.ndarray,
) -> None:
    m1, m2 = mtx1.astype(np.float64), mtx2.astype(np.float64)
    print("\n--- Lens: camera matrix K (mtx) ---")
    print(f"Frobenius ||K1 - K2||: {np.linalg.norm(m1 - m2):.6g}")
    print(f"Relative Frobenius ||K1 - K2|| / ||K1||: {_fro_relative(m1, m2):.6g}")
    print(f"fx: {m1[0,0]:.6g} vs {m2[0,0]:.6g}  (Δ {m2[0,0]-m1[0,0]:.6g})")
    print(f"fy: {m1[1,1]:.6g} vs {m2[1,1]:.6g}  (Δ {m2[1,1]-m1[1,1]:.6g})")
    print(f"cx: {m1[0,2]:.6g} vs {m2[0,2]:.6g}  (Δ {m2[0,2]-m1[0,2]:.6g})")
    print(f"cy: {m1[1,2]:.6g} vs {m2[1,2]:.6g}  (Δ {m2[1,2]-m1[1,2]:.6g})")

    n = max(dist1.size, dist2.size)
    d1 = _pad_dist(dist1, n)
    d2 = _pad_dist(dist2, n)
    print("\n--- Lens: distortion coefficients (dist) ---")
    print(f"vector length (padded to max): {n}")
    print(f"L2 ||dist1 - dist2||: {np.linalg.norm(d1 - d2):.6g}")
    print(f"per-term deltas: {np.array2string(d2 - d1, precision=6)}")
    print(
        "Interpretation: smaller relative K/dist differences mean similar focal length, "
        "principal point, and radial/tangential distortion correction."
    )


def _homography_pixel_to_mm_jacobian(H: np.ndarray, u: float, v: float) -> np.ndarray:
    """
    Jacobian J = d(X_mm, Y_mm) / d(u, v) for the map (u,v) -> (X,Y) via H [u,v,1]^T dehomogenized.
    """
    H = H.astype(np.float64)
    h00, h01, h02 = H[0, 0], H[0, 1], H[0, 2]
    h10, h11, h12 = H[1, 0], H[1, 1], H[1, 2]
    h20, h21, h22 = H[2, 0], H[2, 1], H[2, 2]
    Nx = h00 * u + h01 * v + h02
    Ny = h10 * u + h11 * v + h12
    D = h20 * u + h21 * v + h22
    inv_d2 = 1.0 / (D * D)
    dX_du = (h00 * D - Nx * h20) * inv_d2
    dX_dv = (h01 * D - Nx * h21) * inv_d2
    dY_du = (h10 * D - Ny * h20) * inv_d2
    dY_dv = (h11 * D - Ny * h21) * inv_d2
    return np.array([[dX_du, dX_dv], [dY_du, dY_dv]], dtype=np.float64)


def pixels_per_mm_from_H(
    H: np.ndarray,
    width: int,
    height: int,
    grid: int = 8,
) -> tuple[float, float, float]:
    """
    Average \"pixels per mm\" implied by H at a grid of pixel locations.

    J maps d(u,v) in pixels to d(X,Y) in mm. Singular values of J are mm per pixel
    along principal directions; 1/s_i is px/mm along those directions.
    Returns (mean_px_per_mm, min_px_per_mm, max_px_per_mm) over grid and both singular directions.
    """
    u_edges = np.linspace(0.0, float(width - 1), grid)
    v_edges = np.linspace(0.0, float(height - 1), grid)
    values: list[float] = []
    for u in u_edges:
        for v in v_edges:
            J = _homography_pixel_to_mm_jacobian(H, u, v)
            _, s, _ = np.linalg.svd(J)
            s = np.maximum(s, 1e-15)
            px_mm = 1.0 / s
            values.extend(px_mm.tolist())
    arr = np.asarray(values, dtype=np.float64)
    return float(np.mean(arr)), float(np.min(arr)), float(np.max(arr))


def compare_homographies(
    H1: np.ndarray,
    H2: np.ndarray,
    width: int,
    height: int,
) -> None:
    H1 = H1.astype(np.float64)
    H2 = H2.astype(np.float64)
    n1 = np.linalg.norm(H1, "fro")
    n2 = np.linalg.norm(H2, "fro")
    Hn1 = H1 / (n1 + 1e-15)
    Hn2 = H2 / (n2 + 1e-15)
    print("\n--- Homography H (pixel -> board plane mm) ---")
    print(f"Frobenius ||H1 - H2||: {np.linalg.norm(H1 - H2):.6g}")
    print(
        "Frobenius ||H1/||H1|| - H2/||H2||| (scale-normalized): "
        f"{np.linalg.norm(Hn1 - Hn2):.6g}"
    )
    # Map a grid of undistorted pixels through both H; compare resulting mm positions
    gu, gv = np.meshgrid(
        np.linspace(0, width - 1, 9),
        np.linspace(0, height - 1, 9),
    )
    pts = np.stack([gu.ravel(), gv.ravel()], axis=1).astype(np.float32)
    pts_h = np.hstack([pts, np.ones((pts.shape[0], 1), dtype=np.float32)])
    w1 = (H1 @ pts_h.T).T
    w2 = (H2 @ pts_h.T).T
    p1 = w1[:, :2] / (w1[:, 2:3] + 1e-15)
    p2 = w2[:, :2] / (w2[:, 2:3] + 1e-15)
    diff = np.linalg.norm(p1 - p2, axis=1)
    print(
        f"Grid warp disagreement (9x9, full frame {width}x{height}): "
        f"mean |Δmm|={np.mean(diff):.6g}  max={np.max(diff):.6g}"
    )
    # Cross-check one point with OpenCV
    op1 = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), H1.astype(np.float32)).reshape(-1, 2)
    if np.max(np.abs(op1 - p1)) > 1e-2:
        print("  (warn: manual vs OpenCV perspectiveTransform mismatch > 1e-2)", file=sys.stderr)
    print(
        "Interpretation: lower disagreement means both H agree on mapping pixels to mm across the frame."
    )


def summarize_file(path: Path, width: int, height: int) -> None:
    try:
        d = load_npz(path)
    except Exception as exc:
        print(f"{path}: failed to load ({exc})")
        return
    keys = list(d.keys())
    print(f"\n{path}")
    print(f"  keys: {keys}")
    if "rms" in d:
        print(f"  rms (if present): {float(np.asarray(d['rms']).ravel()[0]):.6g}")
    if "mtx" in d:
        K = d["mtx"].astype(np.float64)
        print(f"  K fx={K[0,0]:.4f} fy={K[1,1]:.4f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
    else:
        print("  (no mtx)")
    if "H" not in d or d["H"] is None:
        print("  H: absent — cannot estimate px/mm from homography.")
        return
    H = np.asarray(d["H"], dtype=np.float64)
    if H.size != 9 or H.shape != (3, 3):
        print("  H: invalid shape — skipping px/mm.")
        return
    mean_pmm, min_pmm, max_pmm = pixels_per_mm_from_H(H, width, height)
    print(
        f"  px/mm from H (SVD of d(mm)/d(px), grid={width}x{height}, sample 8x8): "
        f"mean={mean_pmm:.4f}  min={min_pmm:.4f}  max={max_pmm:.4f}"
    )


def compare_two(path_a: Path, path_b: Path, width: int, height: int) -> int:
    if not path_a.is_file():
        print(f"Not found: {path_a}", file=sys.stderr)
        return 1
    if not path_b.is_file():
        print(f"Not found: {path_b}", file=sys.stderr)
        return 1
    da = load_npz(path_a)
    db = load_npz(path_b)
    for label, d in (("A", da), ("B", db)):
        need = "mtx" in d and "dist" in d
        if not need:
            print(f"File {label} missing mtx or dist.", file=sys.stderr)
            return 1
    print(f"Compare:\n  A: {path_a.resolve()}\n  B: {path_b.resolve()}")
    if "rms" in da:
        print(f"A rms: {float(np.asarray(da['rms']).ravel()[0]):.6g}")
    if "rms" in db:
        print(f"B rms: {float(np.asarray(db['rms']).ravel()[0]):.6g}")

    compare_intrinsics(da["mtx"], da["dist"], db["mtx"], db["dist"])

    Ha = da.get("H")
    Hb = db.get("H")
    if Ha is None or Hb is None:
        print("\nHomography missing in one or both files; skipping H comparison and warp test.")
        return 0
    Ha = np.asarray(Ha, dtype=np.float64)
    Hb = np.asarray(Hb, dtype=np.float64)
    if Ha.shape != (3, 3) or Hb.shape != (3, 3):
        print("\nHomography shape invalid; skipping H comparison.")
        return 0
    compare_homographies(Ha, Hb, width, height)

    print("\n--- Per-file px/mm (same frame size) ---")
    for p, H in ((path_a, Ha), (path_b, Hb)):
        mn, lo, hi = pixels_per_mm_from_H(H, width, height)
        print(f"  {p.name}: mean px/mm={mn:.4f} (min={lo:.4f}, max={hi:.4f})")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Inspect/compare camera_params .npz; scan directory for px/mm from H."
    )
    parser.add_argument(
        "files",
        nargs="*",
        help="Exactly two .npz paths to compare (optional).",
    )
    parser.add_argument(
        "--dir",
        type=Path,
        default=_repo_root() / "config",
        help="Directory to scan for *.npz (default: <repo>/config).",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=_DEFAULT_WIDTH,
        help="Frame width for px/mm sampling and H warp grid (default: %(default)s).",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=_DEFAULT_HEIGHT,
        help="Frame height for px/mm sampling (default: %(default)s).",
    )
    args = parser.parse_args()

    if len(args.files) == 2:
        return compare_two(Path(args.files[0]), Path(args.files[1]), args.width, args.height)
    if len(args.files) == 1 or len(args.files) > 2:
        parser.error("Provide zero files (scan only) or exactly two files to compare.")

    scan_dir = args.dir.resolve()
    if not scan_dir.is_dir():
        print(f"Not a directory: {scan_dir}", file=sys.stderr)
        return 1
    files = sorted(scan_dir.glob("*.npz"))
    if not files:
        print(f"No .npz files in {scan_dir}")
        return 0
    print(f"Scanning {scan_dir} ({len(files)} file(s)), frame {args.width}x{args.height}")
    for f in files:
        summarize_file(f, args.width, args.height)
    print(
        "\nNote: px/mm uses local linearization of H (pixels -> mm). "
        "Use --width/--height to match your calibration image or capture resolution."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
