"""
Monocular Visual SLAM Pipeline
Uses ORB feature detection + tracking to estimate camera pose
and build a sparse 3D map from TUM RGB-D dataset sequences.
"""

import cv2
import numpy as np
import os
import time
import argparse
from pathlib import Path


# ── Camera intrinsics for TUM fr1 sequences ──────────────────────────────────
TUM_FR1_K = np.array([
    [517.3, 0,     318.6],
    [0,     516.5, 255.3],
    [0,     0,     1    ]
], dtype=np.float64)

TUM_FR1_DIST = np.array([0.2624, -0.9531, -0.0054, 0.0026, 1.1633], dtype=np.float64)


class ORBFeatureExtractor:
    def __init__(self, n_features=2000):
        self.orb = cv2.ORB_create(nfeatures=n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    def extract(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        return keypoints, descriptors

    def match(self, desc1, desc2, ratio=0.75):
        if desc1 is None or desc2 is None:
            return []
        matches = self.matcher.knnMatch(desc1, desc2, k=2)
        good = []
        for m, n in matches:
            if m.distance < ratio * n.distance:
                good.append(m)
        return good


class PoseEstimator:
    def __init__(self, K, dist_coeffs):
        self.K = K
        self.dist = dist_coeffs

    def estimate_pose(self, kp1, kp2, matches):
        if len(matches) < 8:
            return None, None, None

        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

        # Essential matrix via RANSAC
        E, mask = cv2.findEssentialMat(
            pts1, pts2, self.K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0
        )

        if E is None:
            return None, None, None

        # Recover rotation and translation
        _, R, t, mask_pose = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask)

        inliers = int(mask_pose.sum())
        return R, t, inliers


class MapPoint:
    _id_counter = 0

    def __init__(self, position):
        self.id = MapPoint._id_counter
        MapPoint._id_counter += 1
        self.position = position  # 3D world coords


class Frame:
    _id_counter = 0

    def __init__(self, img, keypoints, descriptors, pose=None):
        self.id = Frame._id_counter
        Frame._id_counter += 1
        self.img = img
        self.keypoints = keypoints
        self.descriptors = descriptors
        self.pose = pose if pose is not None else np.eye(4)  # 4x4 transform


class SLAMSystem:
    def __init__(self, K, dist_coeffs, visualize=True):
        self.K = K
        self.extractor = ORBFeatureExtractor()
        self.pose_estimator = PoseEstimator(K, dist_coeffs)
        self.visualize = visualize

        self.frames = []
        self.map_points = []
        self.trajectory = []  # List of (x, y, z) camera positions

        # Current global pose (4x4)
        self.current_pose = np.eye(4)

        self.prev_frame = None
        self.initialized = False

        print("[SLAM] System initialized.")
        print(f"[SLAM] Camera intrinsics:\n{K}")

    def process_frame(self, img, frame_id=None):
        t_start = time.time()

        # Extract features
        kp, desc = self.extractor.extract(img)
        frame = Frame(img, kp, desc, pose=self.current_pose.copy())

        if self.prev_frame is None:
            # First frame — set as reference
            self.prev_frame = frame
            self.frames.append(frame)
            pos = self.current_pose[:3, 3]
            self.trajectory.append(pos.copy())
            print(f"[Frame {frame.id}] Reference frame set. Features: {len(kp)}")
            return frame, None

        # Match features with previous frame
        matches = self.extractor.match(self.prev_frame.descriptors, desc)

        if len(matches) < 8:
            print(f"[Frame {frame.id}] Not enough matches ({len(matches)}), skipping.")
            return frame, None

        # Estimate relative pose
        R, t, inliers = self.pose_estimator.estimate_pose(
            self.prev_frame.keypoints, kp, matches
        )

        if R is None:
            print(f"[Frame {frame.id}] Pose estimation failed.")
            return frame, None

        # Update global pose
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()
        self.current_pose = self.current_pose @ np.linalg.inv(T)

        frame.pose = self.current_pose.copy()
        pos = self.current_pose[:3, 3]
        self.trajectory.append(pos.copy())

        elapsed = (time.time() - t_start) * 1000
        print(
            f"[Frame {frame.id:4d}] "
            f"Features: {len(kp):4d} | "
            f"Matches: {len(matches):4d} | "
            f"Inliers: {inliers:4d} | "
            f"Pos: ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) | "
            f"{elapsed:.1f}ms"
        )

        # Visualize
        if self.visualize:
            self._draw_matches(self.prev_frame.img, self.prev_frame.keypoints,
                               img, kp, matches)

        self.prev_frame = frame
        self.frames.append(frame)
        return frame, matches

    def _draw_matches(self, img1, kp1, img2, kp2, matches):
        vis = cv2.drawMatches(
            img1, kp1, img2, kp2,
            matches[:50], None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
        cv2.imshow("ORB Feature Matches", vis)
        cv2.waitKey(1)

    def save_trajectory(self, path="trajectory.txt"):
        with open(path, "w") as f:
            for i, pos in enumerate(self.trajectory):
                f.write(f"{i} {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} 0 0 0 1\n")
        print(f"[SLAM] Trajectory saved to {path} ({len(self.trajectory)} poses)")


def load_tum_images(dataset_path):
    """Load image paths and timestamps from TUM rgb.txt"""
    rgb_file = os.path.join(dataset_path, "rgb.txt")
    images = []

    with open(rgb_file, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) == 2:
                timestamp, img_path = parts
                full_path = os.path.join(dataset_path, img_path)
                images.append((float(timestamp), full_path))

    images.sort(key=lambda x: x[0])
    print(f"[Dataset] Loaded {len(images)} images from {dataset_path}")
    return images


def run_slam(dataset_path, output_dir="results", visualize=True, max_frames=None):
    os.makedirs(output_dir, exist_ok=True)

    # Load images
    images = load_tum_images(dataset_path)
    if max_frames:
        images = images[:max_frames]

    # Init SLAM
    slam = SLAMSystem(TUM_FR1_K, TUM_FR1_DIST, visualize=visualize)

    print(f"\n[SLAM] Processing {len(images)} frames...\n")
    t_total = time.time()

    for i, (timestamp, img_path) in enumerate(images):
        if not os.path.exists(img_path):
            print(f"[Warning] Image not found: {img_path}")
            continue

        img = cv2.imread(img_path)
        if img is None:
            continue

        slam.process_frame(img, frame_id=i)

    elapsed = time.time() - t_total
    fps = len(images) / elapsed
    print(f"\n[SLAM] Done. {len(images)} frames in {elapsed:.1f}s ({fps:.1f} FPS)")

    # Save trajectory
    traj_path = os.path.join(output_dir, "trajectory.txt")
    slam.save_trajectory(traj_path)

    if visualize:
        cv2.destroyAllWindows()

    return slam


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monocular Visual SLAM")
    parser.add_argument("dataset", help="Path to TUM dataset folder (e.g. rgbd_dataset_freiburg1_xyz)")
    parser.add_argument("--output", default="results", help="Output directory")
    parser.add_argument("--no-viz", action="store_true", help="Disable visualization")
    parser.add_argument("--max-frames", type=int, default=None, help="Limit number of frames")
    args = parser.parse_args()

    run_slam(args.dataset, args.output, not args.no_viz, args.max_frames)
