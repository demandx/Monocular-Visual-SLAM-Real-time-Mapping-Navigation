"""
Trajectory Evaluation — ATE & RPE
Evaluates estimated SLAM trajectory against TUM ground truth.
Usage: python evaluate_trajectory.py --gt groundtruth.txt --est results/trajectory.txt
"""

import numpy as np
import argparse
import os


def load_trajectory(filepath):
    """
    Load trajectory file.
    TUM format: timestamp tx ty tz qx qy qz qw
    Our format: index tx ty tz qx qy qz qw
    """
    poses = []
    with open(filepath, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 4:
                t = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                poses.append(t)
    return np.array(poses)


def load_tum_groundtruth(filepath):
    """Load TUM groundtruth.txt — timestamp tx ty tz qx qy qz qw"""
    poses = []
    timestamps = []
    with open(filepath, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) == 8:
                timestamps.append(float(parts[0]))
                t = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                poses.append(t)
    return np.array(poses), timestamps


def align_trajectories(gt, est):
    """
    Align estimated trajectory to ground truth using Umeyama alignment (SVD).
    Returns aligned estimated trajectory.
    """
    n = min(len(gt), len(est))
    gt = gt[:n]
    est = est[:n]

    # Center both
    mu_gt = gt.mean(axis=0)
    mu_est = est.mean(axis=0)

    gt_c = gt - mu_gt
    est_c = est - mu_est

    # SVD
    H = est_c.T @ gt_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Handle reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = mu_gt - R @ mu_est

    # Apply alignment
    est_aligned = (R @ est.T).T + t
    return est_aligned, gt, R, t


def compute_ate(gt, est):
    """
    Absolute Trajectory Error (ATE)
    Measures global consistency between trajectories.
    """
    n = min(len(gt), len(est))
    errors = np.linalg.norm(gt[:n] - est[:n], axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "std": float(np.std(errors)),
        "min": float(np.min(errors)),
        "max": float(np.max(errors)),
        "n_frames": n,
        "errors": errors
    }


def compute_rpe(gt, est, delta=1):
    """
    Relative Pose Error (RPE)
    Measures local accuracy between consecutive frame pairs.
    delta: step size between frame pairs
    """
    n = min(len(gt), len(est))
    trans_errors = []

    for i in range(0, n - delta):
        # Relative translation in GT
        gt_rel = gt[i + delta] - gt[i]
        est_rel = est[i + delta] - est[i]

        # Error in relative translation
        error = np.linalg.norm(gt_rel - est_rel)
        trans_errors.append(error)

    trans_errors = np.array(trans_errors)
    return {
        "rmse": float(np.sqrt(np.mean(trans_errors ** 2))),
        "mean": float(np.mean(trans_errors)),
        "median": float(np.median(trans_errors)),
        "std": float(np.std(trans_errors)),
        "min": float(np.min(trans_errors)),
        "max": float(np.max(trans_errors)),
        "n_pairs": len(trans_errors),
        "errors": trans_errors
    }


def print_results(ate, rpe):
    print("\n" + "=" * 50)
    print("  TRAJECTORY EVALUATION RESULTS")
    print("=" * 50)

    print("\n📍 Absolute Trajectory Error (ATE)")
    print(f"   RMSE   : {ate['rmse'] * 100:.2f} cm")
    print(f"   Mean   : {ate['mean'] * 100:.2f} cm")
    print(f"   Median : {ate['median'] * 100:.2f} cm")
    print(f"   Std    : {ate['std'] * 100:.2f} cm")
    print(f"   Min    : {ate['min'] * 100:.2f} cm")
    print(f"   Max    : {ate['max'] * 100:.2f} cm")
    print(f"   Frames : {ate['n_frames']}")

    print("\n🔁 Relative Pose Error (RPE) — Translation")
    print(f"   RMSE   : {rpe['rmse'] * 100:.2f} cm/frame")
    print(f"   Mean   : {rpe['mean'] * 100:.2f} cm/frame")
    print(f"   Median : {rpe['median'] * 100:.2f} cm/frame")
    print(f"   Pairs  : {rpe['n_pairs']}")
    print("=" * 50)


def save_results(ate, rpe, output_path):
    with open(output_path, "w") as f:
        f.write("metric,value\n")
        f.write(f"ATE_RMSE_m,{ate['rmse']:.6f}\n")
        f.write(f"ATE_mean_m,{ate['mean']:.6f}\n")
        f.write(f"ATE_median_m,{ate['median']:.6f}\n")
        f.write(f"ATE_std_m,{ate['std']:.6f}\n")
        f.write(f"RPE_RMSE_m,{rpe['rmse']:.6f}\n")
        f.write(f"RPE_mean_m,{rpe['mean']:.6f}\n")
        f.write(f"n_frames,{ate['n_frames']}\n")
    print(f"\n[Eval] Results saved to {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate SLAM trajectory")
    parser.add_argument("--gt", required=True, help="Ground truth file (TUM format)")
    parser.add_argument("--est", required=True, help="Estimated trajectory file")
    parser.add_argument("--output", default="results/metrics.csv", help="Save metrics CSV")
    parser.add_argument("--no-align", action="store_true", help="Skip Umeyama alignment")
    args = parser.parse_args()

    print(f"[Eval] Loading ground truth: {args.gt}")
    gt_poses, _ = load_tum_groundtruth(args.gt)

    print(f"[Eval] Loading estimated trajectory: {args.est}")
    est_poses = load_trajectory(args.est)

    print(f"[Eval] GT frames: {len(gt_poses)}, Est frames: {len(est_poses)}")

    if not args.no_align:
        print("[Eval] Aligning trajectories (Umeyama SVD)...")
        est_poses, gt_poses, R, t = align_trajectories(gt_poses, est_poses)

    ate = compute_ate(gt_poses, est_poses)
    rpe = compute_rpe(gt_poses, est_poses)

    print_results(ate, rpe)

    os.makedirs(os.path.dirname(args.output) if os.path.dirname(args.output) else ".", exist_ok=True)
    save_results(ate, rpe, args.output)
