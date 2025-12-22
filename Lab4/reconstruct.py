import numpy as np
import open3d as o3d
from PIL import Image
import argparse
import os

# ============================
# Load RAW depth (uint16)
# ============================
def load_raw_depth(raw_path, width, height):
    depth = np.fromfile(raw_path, dtype=np.uint16)
    depth = depth.reshape((height, width))
    return depth.astype(np.float32)


# ============================
# Convert depth → point cloud
# ============================
def depth_to_pcd(depth, fx, fy, cx, cy):
    h, w = depth.shape

    xs, ys = np.meshgrid(np.arange(w), np.arange(h))

    z = depth / 1000.0       # mm → meters
    x = (xs - cx) * z / fx
    y = (ys - cy) * z / fy

    pts = np.dstack((x, y, z)).reshape(-1, 3)
    pts = pts[pts[:, 2] > 0]  # remove invalid

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    return pcd


# ============================
# Estimate normals
# ============================
def estimate_normals(pcd):
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30)
    )
    pcd.orient_normals_consistent_tangent_plane(50)
    return pcd


# ============================
# Pairwise ICP
# ============================
def pairwise_icp(source, target):
    icp = o3d.pipelines.registration.registration_icp(
        source, target, 
        max_correspondence_distance=0.05,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )
    return icp.transformation


# ============================
# Main
# ============================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pairs", type=str, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    args = parser.parse_args()

    fx, fy = 456.25, 456.18
    cx, cy = 306.34, 251.48

    os.makedirs("outputs", exist_ok=True)

    pcds = []

    # ============================
    # Load all depth files
    # ============================
    with open(args.pairs, "r") as f:
        for line in f:
            raw_path, _ = [x.strip() for x in line.split(",")]

            name = os.path.basename(raw_path).replace(".raw", "")

            depth = load_raw_depth(raw_path, args.width, args.height)
            pcd = depth_to_pcd(depth, fx, fy, cx, cy)

            # Compute normals
            pcd = estimate_normals(pcd)

            # Save individual pcds
            save_path = f"outputs/{name}.ply"
            o3d.io.write_point_cloud(save_path, pcd)
            print(f"Saved {save_path} (points = {len(pcd.points)})")

            pcds.append(pcd)

    # ============================
    # Multi-view ICP registration
    # ============================
    print("\n=== Start ICP Registration ===")

    merged = pcds[0]

    for i in range(1, len(pcds)):
        print(f"Registering cloud {i} ...")

        T = pairwise_icp(pcds[i], merged)
        pcds[i].transform(T)

        merged += pcds[i]

    o3d.io.write_point_cloud("outputs/merged.ply", merged)
    print("\n✅ Complete! Output: outputs/merged.ply")


if __name__ == "__main__":
    main()
