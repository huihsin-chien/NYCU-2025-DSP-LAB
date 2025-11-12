#!/usr/bin/env python3
"""
Part 3 – Edge detection on depth .raw using Sobel and Laplacian
---------------------------------------------------------------
Inputs
  - RAW: uint16 depth image stored row-major without header.
  - CSV: metadata to infer resolution (width, height). If not found, defaults to 640x480.

Outputs (in --outdir)
  - depth_preview.png                : normalized depth
  - sobel_magnitude.png              : Sobel |∇I| (normalized)
  - laplacian_magnitude.png          : |ΔI| (normalized)
  - edge_sobel_bw.png                : binary edge (black background, white edges)
  - edge_laplacian_bw.png            : binary edge (black background, white edges)

Example
  python part3_edges.py --raw 478e...raw --csv 5a07...csv --outdir out --sobel_pct 90 --lap_pct 85
"""

import argparse
import os
import numpy as np
import pandas as pd

from PIL import Image
import matplotlib.pyplot as plt


# -------------------------- Utility --------------------------
def parse_resolution_from_csv(csv_path):
    """
    Try to parse width/height from common CSV headers, else default to 640x480.
    """
    df = pd.read_csv(csv_path)
    w, h = None, None
    for col in df.columns:
        low = str(col).strip().lower()
        if "resolution x" in low or low.endswith("res_x") or low.endswith("width"):
            try:
                w = int(float(df[col].dropna().iloc[0]))
            except Exception:
                pass
        if "resolution y" in low or low.endswith("res_y") or low.endswith("height"):
            try:
                h = int(float(df[col].dropna().iloc[0]))
            except Exception:
                pass
    if w is None or h is None:
        w, h = 640, 480
    return w, h


def read_depth_raw(raw_path, width, height):
    """
    Read uint16 little-endian RAW and reshape to (height, width).
    """
    data = np.fromfile(raw_path, dtype=np.uint16)
    expected = width * height
    if data.size != expected:
        raise ValueError(
            f"Raw size {data.size} != width*height {expected}. "
            "Check CSV metadata or pass --width/--height explicitly."
        )
    return data.reshape((height, width)).astype(np.float32)


def pad_reflect(img, ksize):
    pad = ksize // 2
    return np.pad(img, ((pad, pad), (pad, pad)), mode="reflect"), pad


def conv2d(img, kernel):
    """
    Tiny, dependency-free 2D convolution with reflect padding.
    Good enough for single-run 640x480 with small kernels (3x3, 5x5).
    """
    k = np.array(kernel, dtype=np.float32)
    ks = k.shape[0]
    assert k.shape[0] == k.shape[1] and ks % 2 == 1, "Kernel must be square and odd-sized"
    img_p, pad = pad_reflect(img, ks)
    out = np.zeros_like(img, dtype=np.float32)
    # naive convolution
    for i in range(out.shape[0]):
        for j in range(out.shape[1]):
            region = img_p[i:i+ks, j:j+ks]
            out[i, j] = float((region * k).sum())
    return out


def normalize01(a):
    a = a.astype(np.float32)
    mn, mx = np.nanmin(a), np.nanmax(a)
    if mx > mn:
        return (a - mn) / (mx - mn)
    return np.zeros_like(a, dtype=np.float32)


def percentile_threshold(img, p):
    flat = img[np.isfinite(img)].ravel()
    if flat.size == 0:
        return 0.0
    return np.percentile(flat, p)


def median_3x3_cross(img):
    """
    Very small, fast approximation: average of 4-neighbors (cross).
    Used only to fill invalid zeros in depth.
    """
    kernel = np.array([[0,1,0],[1,0,1],[0,1,0]], dtype=np.float32)
    return conv2d(img, kernel) / 4.0


# -------------------------- Main pipeline --------------------------
def run(raw_path, csv_path=None, width=None, height=None, outdir="out",
        sobel_pct=90.0, lap_pct=85.0, lap8=False):
    if width is None or height is None:
        if csv_path:
            width_csv, height_csv = parse_resolution_from_csv(csv_path)
            width = width or width_csv
            height = height or height_csv
        else:
            width, height = 640, 480

    os.makedirs(outdir, exist_ok=True)

    depth = read_depth_raw(raw_path, width, height)

    # Replace invalid zeros by local cross-average to avoid creating false edges
    invalid = (depth <= 0)
    if np.any(invalid):
        fill = median_3x3_cross(depth)
        depth[invalid] = fill[invalid]

    # -------- Sobel --------
    Kx = [[-1, 0, 1],
          [-2, 0, 2],
          [-1, 0, 1]]
    Ky = [[-1, -2, -1],
          [ 0,  0,  0],
          [ 1,  2,  1]]
    gx = conv2d(depth, Kx)
    gy = conv2d(depth, Ky)
    grad_mag = np.sqrt(gx*gx + gy*gy)

    thr_sobel = percentile_threshold(grad_mag, sobel_pct)
    edge_sobel = (grad_mag >= thr_sobel).astype(np.uint8) * 255

    # -------- Laplacian --------
    if lap8:
        Lap = [[1, 1, 1],
               [1,-8, 1],
               [1, 1, 1]]
    else:
        Lap = [[0, 1, 0],
               [1,-4, 1],
               [0, 1, 0]]
    lap = conv2d(depth, Lap)
    lap_abs = np.abs(lap)

    thr_lap = percentile_threshold(lap_abs, lap_pct)
    edge_lap = (lap_abs >= thr_lap).astype(np.uint8) * 255

    # -------- Save previews --------
    depth_vis = (normalize01(depth) * 255).astype(np.uint8)
    sobel_mag_vis = (normalize01(grad_mag) * 255).astype(np.uint8)
    lap_mag_vis   = (normalize01(lap_abs) * 255).astype(np.uint8)

    plt.figure(figsize=(6,4)); plt.title("Depth (normalized)"); plt.imshow(depth_vis); plt.axis("off")
    plt.savefig(os.path.join(outdir, "depth_preview.png"), bbox_inches="tight", pad_inches=0); plt.close()

    plt.figure(figsize=(6,4)); plt.title("Sobel gradient magnitude (normalized)"); plt.imshow(sobel_mag_vis); plt.axis("off")
    plt.savefig(os.path.join(outdir, "sobel_magnitude.png"), bbox_inches="tight", pad_inches=0); plt.close()

    plt.figure(figsize=(6,4)); plt.title("Laplacian magnitude (normalized)"); plt.imshow(lap_mag_vis); plt.axis("off")
    plt.savefig(os.path.join(outdir, "laplacian_magnitude.png"), bbox_inches="tight", pad_inches=0); plt.close()

    # -------- Save binary edges --------
    Image.fromarray(edge_sobel).save(os.path.join(outdir, "edge_sobel_bw.png"))
    Image.fromarray(edge_lap).save(os.path.join(outdir, "edge_laplacian_bw.png"))

    return {
        "depth_preview": os.path.join(outdir, "depth_preview.png"),
        "sobel_mag": os.path.join(outdir, "sobel_magnitude.png"),
        "lap_mag": os.path.join(outdir, "laplacian_magnitude.png"),
        "edge_sobel": os.path.join(outdir, "edge_sobel_bw.png"),
        "edge_lap": os.path.join(outdir, "edge_laplacian_bw.png")
    }


def build_argparser():
    ap = argparse.ArgumentParser(description="Part 3 edge detection (Sobel & Laplacian) for depth RAW")
    ap.add_argument("--raw", required=True, help="path to depth .raw (uint16)")
    ap.add_argument("--csv", default=None, help="path to metadata .csv (to infer resolution)")
    ap.add_argument("--width", type=int, default=None, help="override width (if missing/incorrect in CSV)")
    ap.add_argument("--height", type=int, default=None, help="override height (if missing/incorrect in CSV)")
    ap.add_argument("--outdir", default="out", help="output directory")
    ap.add_argument("--sobel_pct", type=float, default=90.0, help="percentile threshold for Sobel magnitude (0-100)")
    ap.add_argument("--lap_pct", type=float, default=85.0, help="percentile threshold for Laplacian magnitude (0-100)")
    ap.add_argument("--lap8", action="store_true", help="use 8-neighbor Laplacian (1,-8,1; full 8-neighborhood)")
    return ap


def main():
    args = build_argparser().parse_args()
    run(args.raw, args.csv, args.width, args.height, args.outdir,
        sobel_pct=args.sobel_pct, lap_pct=args.lap_pct, lap8=args.lap8)


if __name__ == "__main__":
    main()
