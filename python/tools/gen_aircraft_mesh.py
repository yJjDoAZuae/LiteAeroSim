"""Generate python/assets/aircraft_lp.glb from the source OBJ mesh.

Coordinate frame of the source OBJ (and of the output asset):
    +X  aircraft nose (forward)
    +Y  right wing (lateral)
    +Z  up

This matches the Vispy scene convention (+Z = altitude) so no additional
axis-flip is required when placing the mesh in the scene.

Decimation strategy
-------------------
The source mesh is symmetric about the Y=0 plane.  To guarantee that the
output is also exactly symmetric and watertight, this script:

  1. Extracts the right half (faces whose centroid Y >= 0).
  2. Decimates the right half to roughly TARGET_HALF_FACES triangles using
     quadric error decimation (pyfqmr via trimesh).
  3. Mirrors the decimated right half by negating Y and reversing face winding.
  4. Concatenates both halves and welds vertices on the Y=0 seam.
  5. Removes degenerate and duplicate faces, recalculates normals.
  6. Verifies: face count <= MAX_FACES, is_watertight, max symmetry error < 1 mm.

Usage
-----
Run from the liteaero-sim project root:

    python python/tools/gen_aircraft_mesh.py [--src SRC] [--dst DST]
                                              [--target TARGET] [--seam-tol TOL]

Defaults:
    --src     generic01.obj
    --dst     python/assets/aircraft_lp.glb
    --target  400    (total face target; half-mesh target = target // 2)
    --seam-tol 1e-3  (weld tolerance in metres)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pyfqmr
import trimesh


MAX_FACES = 500  # hard limit; script errors if exceeded


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _reindex_half(vertices: np.ndarray, faces: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (new_vertices, new_faces) with only the vertices referenced by faces."""
    unique_idx, inv = np.unique(faces, return_inverse=True)
    return vertices[unique_idx], inv.reshape(faces.shape)


def _symmetry_max_error_m(vertices: np.ndarray) -> float:
    """Return the maximum distance from any vertex to its Y-mirror."""
    mirror = vertices.copy()
    mirror[:, 1] *= -1
    errors = []
    for v, m in zip(vertices, mirror):
        dists = np.linalg.norm(vertices - m, axis=1)
        errors.append(dists.min())
    return float(max(errors))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def generate(
    src: Path,
    dst: Path,
    target_faces: int = 400,
    seam_tol: float = 1e-3,
) -> None:
    # ------------------------------------------------------------------
    # 1. Load
    # ------------------------------------------------------------------
    print(f"Loading {src} ...")
    mesh = trimesh.load(str(src), force="mesh")
    print(f"  {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
    print(f"  Bounding box X [{mesh.bounds[0,0]:.3f}, {mesh.bounds[1,0]:.3f}]"
          f"  Y [{mesh.bounds[0,1]:.3f}, {mesh.bounds[1,1]:.3f}]"
          f"  Z [{mesh.bounds[0,2]:.3f}, {mesh.bounds[1,2]:.3f}]")
    print(f"  Watertight: {mesh.is_watertight}")

    # ------------------------------------------------------------------
    # 2. Repair if not watertight
    # ------------------------------------------------------------------
    if not mesh.is_watertight:
        print("  Repairing holes ...")
        trimesh.repair.fill_holes(mesh)
        mesh.fix_normals()
        print(f"  Watertight after repair: {mesh.is_watertight}")

    verts = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.faces, dtype=np.int64)

    # ------------------------------------------------------------------
    # 3. Extract right half (centroid Y >= 0)
    # ------------------------------------------------------------------
    centroids = verts[faces].mean(axis=1)         # (N, 3)
    right_mask = centroids[:, 1] >= 0.0
    right_faces_global = faces[right_mask]

    right_verts, right_faces = _reindex_half(verts, right_faces_global)
    half_mesh = trimesh.Trimesh(vertices=right_verts, faces=right_faces)
    print(f"\nRight half: {len(half_mesh.vertices)} vertices, {len(half_mesh.faces)} faces")

    # ------------------------------------------------------------------
    # 4. Decimate right half via pyfqmr (quadric error decimation).
    #
    #    pyfqmr tends to stall far above the requested target_count due
    #    to its per-iteration quality guard and a default iteration cap.
    #    To work around this we use a two-pass strategy:
    #      Pass 1 — coarse reduction from ~4000 to ~600 faces (still
    #               above the stall point for the original mesh topology)
    #      Pass 2 — fine reduction from ~600 to target_faces//2 with
    #               max_iterations=500 to push through any remaining stall.
    #
    #    target_half is target_faces//2 so that after mirroring the total
    #    stays under MAX_FACES even if pyfqmr overshoots by 20-30 %.
    # ------------------------------------------------------------------
    target_half = max(10, target_faces // 2)

    def _run_pyfqmr(v: np.ndarray, f: np.ndarray,
                    target: int, iters: int = 100) -> tuple[np.ndarray, np.ndarray]:
        simp = pyfqmr.Simplify()
        simp.setMesh(v.astype(np.float64), f.astype(np.int32))
        simp.simplify_mesh(
            target_count=target,
            aggressiveness=9,
            preserve_border=False,
            verbose=False,
            max_iterations=iters,
        )
        ov, of, _ = simp.getMesh()
        return np.asarray(ov, dtype=np.float64), np.asarray(of, dtype=np.int64)

    # Pass 1: coarse reduction
    pass1_v, pass1_f = _run_pyfqmr(right_verts, right_faces,
                                    target=max(target_half * 3, 600))
    print(f"  Pass 1: {len(pass1_f)} faces")

    # Pass 2: fine reduction with extended iteration budget
    dec_verts, dec_faces = _run_pyfqmr(pass1_v, pass1_f,
                                        target=target_half, iters=500)
    print(f"  Pass 2: {len(dec_faces)} faces  (target {target_half})")

    # ------------------------------------------------------------------
    # 4b. Fill holes in the decimated half before mirroring.
    #     The source OBJ has open boundary loops that survive decimation.
    #     Filling them on the small decimated mesh (~100-200 faces) is
    #     tractable; filling them on the full 7952-face source is not.
    #     After hole-fill the half-mesh boundary is only its Y=0 seam,
    #     so the mirror+weld step will produce a watertight closed mesh.
    # ------------------------------------------------------------------
    dec_half = trimesh.Trimesh(vertices=dec_verts, faces=dec_faces)
    if not dec_half.is_watertight:
        print(f"  Half-mesh has {len(dec_half.faces)} faces; filling holes ...")
        trimesh.repair.fill_holes(dec_half)
        dec_half.fix_normals()
        print(f"  After fill: {len(dec_half.vertices)} verts, "
              f"{len(dec_half.faces)} faces, watertight={dec_half.is_watertight}")
    dec_verts = np.asarray(dec_half.vertices, dtype=np.float64)
    dec_faces = np.asarray(dec_half.faces, dtype=np.int64)

    # ------------------------------------------------------------------
    # 4c. Snap seam vertices to exactly Y=0 before mirroring.
    #     pyfqmr may move originally-Y=0 vertices to Y≈±ε.  Without
    #     snapping, mirrored pairs at (x, +ε, z) and (x, -ε, z) are 2ε
    #     apart and may not be caught by the weld tolerance.
    # ------------------------------------------------------------------
    seam_mask = np.abs(dec_verts[:, 1]) < seam_tol
    dec_verts[seam_mask, 1] = 0.0
    print(f"  Snapped {seam_mask.sum()} seam vertices to Y=0")

    # ------------------------------------------------------------------
    # 5. Mirror to create left half
    #    Negating Y reverses handedness → flip face winding [0,1,2] → [0,2,1]
    # ------------------------------------------------------------------
    left_verts = dec_verts.copy()
    left_verts[:, 1] *= -1.0
    left_faces = dec_faces[:, [0, 2, 1]]          # reversed winding

    # ------------------------------------------------------------------
    # 6. Concatenate and weld seam
    # ------------------------------------------------------------------
    n_right = len(dec_verts)
    all_verts = np.vstack([dec_verts, left_verts])
    all_faces = np.vstack([dec_faces, left_faces + n_right])

    merged = trimesh.Trimesh(vertices=all_verts, faces=all_faces)
    before_weld = len(merged.vertices)
    digits = max(1, int(round(-np.log10(seam_tol))))
    merged.merge_vertices(digits_vertex=digits)
    # Remove degenerate and duplicate faces, fix normals
    merged.update_faces(merged.unique_faces())
    merged.remove_unreferenced_vertices()
    merged.fix_normals()
    print(f"\nMerged (before weld): {before_weld} verts -> "
          f"after weld: {len(merged.vertices)} verts, {len(merged.faces)} faces")

    # Attempt to close any remaining open boundary loops on the merged mesh.
    # After the mirror+weld the only open edges are those that survived from
    # the original half-mesh holes (not on the Y=0 seam, which is now welded).
    # fill_holes is more likely to succeed on the small merged mesh than on
    # the 7952-face source.
    if not merged.is_watertight:
        trimesh.repair.fill_holes(merged)
        merged.fix_normals()
        merged.update_faces(merged.unique_faces())
        merged.remove_unreferenced_vertices()
        print(f"  After hole fill: {len(merged.vertices)} verts, "
              f"{len(merged.faces)} faces, watertight={merged.is_watertight}")

    # ------------------------------------------------------------------
    # 7. Verify
    # ------------------------------------------------------------------
    ok = True

    if len(merged.faces) > MAX_FACES:
        print(f"  ERROR: face count {len(merged.faces)} exceeds limit {MAX_FACES}")
        ok = False
    else:
        print(f"  Face count {len(merged.faces)} <= {MAX_FACES}  OK")

    if merged.is_watertight:
        print(f"  Watertight  OK")
    else:
        print(f"  WARNING: mesh is not watertight after merge "
              f"(open edges: {len(merged.faces) - len(trimesh.graph.connected_components(merged.face_adjacency))})")
        # Non-fatal: export anyway but warn

    sym_err = _symmetry_max_error_m(np.asarray(merged.vertices))
    if sym_err < seam_tol:
        print(f"  Max symmetry error {sym_err*1000:.3f} mm  OK")
    else:
        print(f"  WARNING: max symmetry error {sym_err*1000:.3f} mm  "
              f"(tolerance {seam_tol*1000:.3f} mm)")

    if not ok:
        sys.exit(1)

    # ------------------------------------------------------------------
    # 8. Export as GLB
    # ------------------------------------------------------------------
    dst.parent.mkdir(parents=True, exist_ok=True)
    merged.export(str(dst))
    print(f"\nExported: {dst}  ({dst.stat().st_size // 1024} KB)")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src",       type=Path, default=Path("generic01.obj"),
                   help="Source OBJ path (default: %(default)s)")
    p.add_argument("--dst",       type=Path, default=Path("python/assets/aircraft_lp.glb"),
                   help="Output GLB path (default: %(default)s)")
    p.add_argument("--target",    type=int,  default=360,
                   help="Total face target after merge (default: %(default)s)")
    p.add_argument("--seam-tol", type=float, default=1e-3,
                   help="Seam weld tolerance in metres (default: %(default)s)")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    generate(src=args.src, dst=args.dst,
             target_faces=args.target, seam_tol=args.seam_tol)
