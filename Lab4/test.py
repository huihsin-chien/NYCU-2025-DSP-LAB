#!/usr/bin/env python3
"""
depth_to_pointcloud.py

Usage:
  python depth_to_pointcloud.py --pairs pairs.txt --width 640 --height 480 --fx 456.25 --fy 456.175781 --cx 306.34375 --cy 251.482422

pairs.txt format (one pair per line, depth_raw_path,color_png_path):
LiDAR_data_cp/lidar_desk_Depth.raw,LiDAR_data_cp/lidar_desk_Depth.png
...

This script:
 - Tries to read depth raw as uint16/uint32/float32/uint8 (and a byteswapped uint16 fallback).
 - Converts depth to meters if values look like millimeters (>10).
 - Projects to 3D using intrinsics fx,fy,cx,cy.
 - Saves per-view colored PLY files to ./outputs/.
 - If open3d is installed, attempts pairwise ICP registration and saves merged_cloud.ply.
 - Optional: saves normals and can run Poisson reconstruction if requested (needs open3d).
"""

import argparse
from pathlib import Path
import numpy as np
from PIL import Image
import sys

def guess_and_read_raw(path, expected_shape):
    b = Path(path).read_bytes()
    for dtype in [np.uint16, np.uint32, np.float32, np.uint8]:
        try:
            arr = np.frombuffer(b, dtype=dtype)
            if arr.size == expected_shape[0]*expected_shape[1]:
                return arr.reshape(expected_shape).astype(np.float32), dtype.__name__
        except Exception:
            pass
    # try byteswapped uint16
    try:
        arr = np.frombuffer(b, dtype=np.uint16).byteswap().reshape(expected_shape)
        return arr.astype(np.float32), "uint16_byteswapped"
    except Exception:
        raise ValueError("Unable to interpret raw file with tried dtypes/sizes. If your raw has a header or different resolution, provide --offset, --width, --height or adjust pairs file.")

def depth_to_points(depth, color_img, fx, fy, cx, cy):
    h,w = depth.shape
    maxd = float(np.nanmax(depth))
    if maxd > 10.0:
        depth_m = depth / 1000.0
        units = "mm->m"
    else:
        depth_m = depth.copy()
        units = "m"
    u = np.arange(w)
    v = np.arange(h)
    uu, vv = np.meshgrid(u, v)
    z = depth_m
    valid = (z > 0) & np.isfinite(z)
    x = (uu - cx) * z / fx
    y = (vv - cy) * z / fy
    pts = np.stack([x, y, z], axis=-1)[valid]
    colors = None
    if color_img is not None:
        color = np.asarray(color_img.convert("RGB"))
        if color.shape[:2] != (h,w):
            color = np.array(Image.fromarray(color).resize((w,h)))
        cols = color.reshape(-1,3)[valid.reshape(-1)]
        colors = (cols.astype(np.uint8))
    return pts, colors, units

def write_ply_ascii(path, pts, colors=None):
    path = Path(path)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {pts.shape[0]}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        if colors is None:
            for p in pts:
                f.write(f"{p[0]} {p[1]} {p[2]} 255 255 255\n")
        else:
            for p,c in zip(pts, colors):
                f.write(f"{p[0]} {p[1]} {p[2]} {int(c[0])} {int(c[1])} {int(c[2])}\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pairs", required=True, help="CSV or text file with lines: depth.raw,color.png")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fx", type=float, default=456.25)
    parser.add_argument("--fy", type=float, default=456.175781)
    parser.add_argument("--cx", type=float, default=306.34375)
    parser.add_argument("--cy", type=float, default=251.482422)
    parser.add_argument("--offset", type=int, default=0, help="byte offset to skip at start of raw file (header)")
    parser.add_argument("--out", default="outputs")
    parser.add_argument("--no_register", action="store_true", help="skip ICP registration even if open3d is installed")
    args = parser.parse_args()

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    expected_shape = (args.height, args.width)

    pairs = []
    with open(args.pairs, "r") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            parts = [x.strip() for x in s.split(",")]
            if len(parts) == 1:
                parts.append("")  # color optional
            pairs.append((parts[0], parts[1]))

    generated = []
    failures = []
    for depth_path, color_path in pairs:
        depth_path = Path(depth_path)
        try:
            # handle offset by reading bytes and slicing
            raw_bytes = depth_path.read_bytes()
            if args.offset:
                raw_bytes = raw_bytes[args.offset:]
            # attempt reading from the sliced bytes
            # we'll create a temp file object in memory by np.frombuffer on raw_bytes
            # reuse guess_and_read_raw by writing temp bytes to a BytesIO? simpler: replicate logic here
            arr = None
            dtype_name = None
            for dtype in [np.uint16, np.uint32, np.float32, np.uint8]:
                try:
                    temp = np.frombuffer(raw_bytes, dtype=dtype)
                    if temp.size == expected_shape[0]*expected_shape[1]:
                        arr = temp.reshape(expected_shape).astype(np.float32)
                        dtype_name = dtype.__name__
                        break
                except Exception:
                    pass
            if arr is None:
                try:
                    temp = np.frombuffer(raw_bytes, dtype=np.uint16).byteswap().reshape(expected_shape)
                    arr = temp.astype(np.float32)
                    dtype_name = "uint16_byteswapped"
                except Exception:
                    raise ValueError("Could not interpret raw with the given width/height/offset. Check params.")
            color_img = None
            if color_path:
                color_p = Path(color_path)
                if color_p.exists():
                    color_img = Image.open(str(color_p)).convert("RGB")
            pts, colors, units = depth_to_points(arr, color_img, args.fx, args.fy, args.cx, args.cy)
            outname = outdir / (depth_path.stem + ".ply")
            write_ply_ascii(outname, pts, colors)
            print(f"Saved {outname} (dtype guessed: {dtype_name}, units assumption: {units}, points: {pts.shape[0]})")
            generated.append(str(outname))
        except Exception as e:
            print(f"Failed {depth_path}: {e}")
            failures.append((str(depth_path), str(e)))

    # Try registration with open3d if available and not skipped
    if (not args.no_register) and generated:
        try:
            import open3d as o3d
            print("Open3D detected — attempting pairwise ICP registration...")
            pcds = []
            for p in generated:
                pcd = o3d.io.read_point_cloud(p)
                pcd_down = pcd.voxel_down_sample(voxel_size=0.01)
                pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
                pcds.append((pcd, pcd_down))
            base = pcds[0][0]
            base_down = pcds[0][1]
            merged = base
            for i in range(1, len(pcds)):
                src_full, src_down = pcds[i]
                threshold = 0.05
                res = o3d.pipelines.registration.registration_icp(
                    src_down, base_down, threshold, np.eye(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane()
                )
                T = res.transformation
                res_ref = o3d.pipelines.registration.registration_icp(
                    src_full, base, threshold*0.5, T,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane()
                )
                src_full.transform(res_ref.transformation)
                merged += src_full
                print(f"Aligned view {i}: fitness={res_ref.fitness:.4f}, rmse={res_ref.inlier_rmse:.4f}")
            merged_down = merged.voxel_down_sample(voxel_size=0.002)
            merged_path = outdir / "merged_cloud.ply"
            o3d.io.write_point_cloud(str(merged_path), merged_down)
            print(f"Merged saved: {merged_path}")
        except Exception as e:
            print("Open3D registration failed or Open3D not installed. Exception:", e)

    print("Done. Per-view PLYs are in:", outdir)
    if failures:
        print("Failures:", failures)

if __name__ == "__main__":
    main()
