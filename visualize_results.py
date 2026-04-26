"""
Trajectory Visualization
Plots estimated vs ground truth trajectory in 3D and 2D.
Usage: python visualize_results.py --gt groundtruth.txt --est results/trajectory.txt
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os


def load_trajectory(filepath):
    poses = []
    with open(filepath, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 4:
                poses.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(poses)


def load_groundtruth(filepath):
    poses = []
    with open(filepath, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) == 8:
                poses.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(poses)


def align(gt, est):
    n = min(len(gt), len(est))
    gt, est = gt[:n], est[:n]
    mu_gt, mu_est = gt.mean(0), est.mean(0)
    H = (est - mu_est).T @ (gt - mu_gt)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = mu_gt - R @ mu_est
    return (R @ est.T).T + t, gt


def plot_trajectory(gt, est, output_dir="results"):
    os.makedirs(output_dir, exist_ok=True)
    n = min(len(gt), len(est))
    gt, est = gt[:n], est[:n]

    # ── 3D Plot ───────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 6))

    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(gt[:, 0], gt[:, 1], gt[:, 2], 'b-', linewidth=1.5, label='Ground Truth')
    ax1.plot(est[:, 0], est[:, 1], est[:, 2], 'r--', linewidth=1.5, label='Estimated')
    ax1.scatter(*gt[0], color='green', s=80, zorder=5, label='Start')
    ax1.scatter(*gt[-1], color='red', s=80, zorder=5, label='End')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory')
    ax1.legend()

    # ── 2D Top-down Plot ──────────────────────────────────────
    ax2 = fig.add_subplot(122)
    ax2.plot(gt[:, 0], gt[:, 2], 'b-', linewidth=1.5, label='Ground Truth')
    ax2.plot(est[:, 0], est[:, 2], 'r--', linewidth=1.5, label='Estimated')
    ax2.scatter(gt[0, 0], gt[0, 2], color='green', s=80, zorder=5, label='Start')
    ax2.scatter(gt[-1, 0], gt[-1, 2], color='red', s=80, zorder=5, label='End')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Z (m)')
    ax2.set_title('Top-down View (X-Z Plane)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    plt.tight_layout()
    out = os.path.join(output_dir, "trajectory_plot.png")
    plt.savefig(out, dpi=150)
    print(f"[Viz] Trajectory plot saved: {out}")
    plt.show()

    # ── ATE Error over time ───────────────────────────────────
    errors = np.linalg.norm(gt - est, axis=1)
    fig2, ax = plt.subplots(figsize=(10, 4))
    ax.plot(errors * 100, color='crimson', linewidth=1)
    ax.fill_between(range(len(errors)), errors * 100, alpha=0.2, color='crimson')
    ax.axhline(np.mean(errors) * 100, color='navy', linestyle='--',
               label=f'Mean ATE: {np.mean(errors)*100:.2f} cm')
    ax.set_xlabel('Frame')
    ax.set_ylabel('ATE Error (cm)')
    ax.set_title('Absolute Trajectory Error over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out2 = os.path.join(output_dir, "ate_error.png")
    plt.savefig(out2, dpi=150)
    print(f"[Viz] ATE error plot saved: {out2}")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--gt",  required=True, help="Ground truth file (TUM format)")
    parser.add_argument("--est", required=True, help="Estimated trajectory file")
    parser.add_argument("--output", default="results", help="Output directory for plots")
    parser.add_argument("--no-align", action="store_true", help="Skip alignment")
    args = parser.parse_args()

    gt  = load_groundtruth(args.gt)
    est = load_trajectory(args.est)

    if not args.no_align:
        est, gt = align(gt, est)

    plot_trajectory(gt, est, args.output)
