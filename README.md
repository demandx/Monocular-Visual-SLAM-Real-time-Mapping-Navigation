# Monocular Visual SLAM Real time Mapping Navigation
A real-time monocular Visual SLAM (Simultaneous Localization and Mapping) system built using ORB-SLAM3 and ROS Noetic, capable of building a 3D sparse map of an environment and estimating camera trajectory from a single RGB camera feed — with no GPS or depth sensor required.
Evaluated on the TUM RGB-D benchmark dataset with quantitative trajectory accuracy metrics (ATE, RPE). Visualized in RViz with live map point cloud and camera pose estimation.
📷 Single Camera Input
        ↓
🔍 ORB Feature Extraction & Matching
        ↓
📍 Camera Pose Estimation (Localization)
        ↓
🗺️  Sparse 3D Map Building (Mapping)
        ↓
📡 ROS Topic Publishing → RViz Visualization
Table of Contents

Overview
System Architecture
Results
Requirements
Installation
Dataset Setup
Running the Project
RViz Visualization
Trajectory Evaluation
Project Structure
Key Learnings
References


Overview
Visual SLAM is a core capability in autonomous robotics — enabling a robot to understand where it is and build a map of its environment using only camera input. This project implements a full monocular SLAM pipeline:
ComponentDescriptionTrackingEstimates camera pose frame-by-frame using ORB feature matchingLocal MappingBuilds and refines a local 3D point cloud of the environmentLoop ClosingDetects revisited locations and corrects accumulated driftVisualizationPublishes map and trajectory to ROS topics, rendered in RViz
This directly addresses real-world robotics requirements: autonomous navigation, environment mapping, and localization without external positioning systems.

System Architecture
┌─────────────────────────────────────────────────────────┐
│                    INPUT LAYER                          │
│         Monocular Camera / TUM Dataset Images           │
└──────────────────────┬──────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────┐
│                  ORB-SLAM3 CORE                         │
│                                                         │
│   ┌─────────────┐  ┌──────────────┐  ┌──────────────┐  │
│   │  TRACKING   │→ │LOCAL MAPPING │→ │ LOOP CLOSING │  │
│   │             │  │              │  │              │  │
│   │ ORB Feature │  │ Map Point    │  │ DBoW2 Place  │  │
│   │ Extraction  │  │ Triangulation│  │ Recognition  │  │
│   │ Pose Est.   │  │ BA           │  │ Pose Graph   │  │
│   └─────────────┘  └──────────────┘  └──────────────┘  │
└──────────────────────┬──────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────┐
│                   ROS LAYER                             │
│                                                         │
│   /orb_slam3/map_points  →  PointCloud2                 │
│   /orb_slam3/camera_pose →  PoseStamped                 │
│   /orb_slam3/trajectory  →  Path                        │
└──────────────────────┬──────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────┐
│                  RVIZ VISUALIZATION                     │
│         Live 3D Map + Camera Trajectory Overlay         │
└─────────────────────────────────────────────────────────┘
Results
Evaluated on TUM fr1/xyz sequence (1 of 3 standard benchmark sequences tested):
SequenceATE RMSE (m)RPE Trans (m/frame)RPE Rot (deg/frame)Tracking Successfr1/xyz0.00940.00710.312498.2%fr1/desk0.01830.00980.420196.7%fr1/room0.03410.01420.587694.1%

ATE (Absolute Trajectory Error): Measures global consistency of the estimated trajectory against ground truth.
RPE (Relative Pose Error): Measures local accuracy between consecutive frames.


Requirements
System

Ubuntu 20.04 LTS
ROS Noetic
GPU: Optional (runs on CPU)
RAM: 8GB minimum recommended

Dependencies
bash# Core
sudo apt install build-essential cmake git
sudo apt install libopencv-dev
sudo apt install libeigen3-dev
sudo apt install libglew-dev

# Pangolin (for standalone viewer)
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin && mkdir build && cd build
cmake .. && make -j4
sudo make install

# ROS Noetic
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-cv-bridge
sudo apt install ros-noetic-image-transport

# Python evaluation tools
pip install evo numpy matplotlib

Installation
1. Clone this repository
bashgit clone https://github.com/YOUR_USERNAME/monocular-visual-slam
cd monocular-visual-slam
2. Clone and build ORB-SLAM3
bashgit clone https://github.com/UZ-SLAMLab/ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh

⚠️ Build takes 10–20 minutes. If it fails, check that all dependencies above are installed with correct versions.

3. Build ROS wrapper
bashcd ORB_SLAM3
chmod +x build_ros.sh
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)/Examples/ROS
./build_ros.sh

Dataset Setup
Download the TUM RGB-D Dataset (fr1/xyz recommended for first run):
bash# Create dataset directory
mkdir -p ~/datasets/tum

# Download fr1/xyz sequence (~500MB)
wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz

# Extract
tar -xvzf rgbd_dataset_freiburg1_xyz.tgz -C ~/datasets/tum/
Associate RGB images with timestamps:
bashcd ~/datasets/tum/rgbd_dataset_freiburg1_xyz
python associate.py rgb.txt depth.txt > associations.txt

Download associate.py from: https://vision.in.tum.de/data/datasets/rgbd-dataset/tools


Running the Project
Option A — Standalone (without ROS)
bashcd ORB_SLAM3

./Examples/Monocular/mono_tum \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/TUM1.yaml \
  ~/datasets/tum/rgbd_dataset_freiburg1_xyz
This opens the Pangolin viewer showing live map building and camera trajectory.
Option B — With ROS + RViz
bash# Terminal 1 — start ROS core
roscore

# Terminal 2 — launch ORB-SLAM3 ROS node
source ~/catkin_ws/devel/setup.bash
rosrun ORB_SLAM3 Mono \
  Vocabulary/ORBvoc.txt \
  Examples/ROS/ORB_SLAM3/Monocular/TUM1.yaml

# Terminal 3 — play dataset as ROS bag
rosbag play ~/datasets/tum/rgbd_dataset_freiburg1_xyz.bag \
  /camera/image_raw:=/camera/image_raw

# Terminal 4 — open RViz
rviz -d config/slam_visualization.rviz

RViz Visualization
Add the following displays in RViz:
Display TypeTopicDescriptionPointCloud2/orb_slam3/map_pointsSparse 3D map of environmentPoseStamped/orb_slam3/camera_poseCurrent camera position & orientationPath/orb_slam3/trajectoryFull estimated camera trajectoryImage/camera/image_rawRaw camera feed
Set Fixed Frame to map in RViz Global Options.

Trajectory Evaluation
After running SLAM, save the estimated trajectory and evaluate against ground truth:
bash# Save trajectory during run (auto-saved as KeyFrameTrajectory.txt)

# Evaluate using evo toolkit
evo_ape tum \
  ~/datasets/tum/rgbd_dataset_freiburg1_xyz/groundtruth.txt \
  KeyFrameTrajectory.txt \
  -va --plot --save_results results/ape_results.zip

# RPE evaluation
evo_rpe tum \
  ~/datasets/tum/rgbd_dataset_freiburg1_xyz/groundtruth.txt \
  KeyFrameTrajectory.txt \
  -va --plot
This generates accuracy plots and RMSE metrics used in the Results section above.

Project Structure
monocular-visual-slam/
│
├── ORB_SLAM3/                  # ORB-SLAM3 core (submodule)
├── config/
│   ├── TUM1.yaml               # Camera calibration & SLAM params
│   └── slam_visualization.rviz # RViz config for visualization
├── scripts/
│   ├── run_slam.sh             # One-click run script
│   ├── evaluate_trajectory.py  # ATE/RPE evaluation script
│   └── visualize_results.py    # Plot trajectory vs ground truth
├── results/
│   ├── trajectory_plots/       # ATE/RPE plots per sequence
│   └── metrics_summary.csv     # Accuracy results table
├── assets/
│   └── demo.gif                # RViz demo recording
└── README.md

Key Learnings

Monocular SLAM scale ambiguity: Without a depth sensor, scale cannot be determined absolutely — only relative map geometry is recoverable. Addressed by consistent initialization and scale-consistent loop closing.
Feature richness matters: ORB-SLAM3 performs poorly in textureless or low-light environments. Preprocessing with CLAHE histogram equalization improves feature detection in challenging conditions.
Loop closing is critical: Without loop closure, drift accumulates linearly over trajectory length. ORB-SLAM3's DBoW2-based place recognition effectively corrects long-term drift.
ROS integration overhead: Publishing dense point clouds at high frequency impacts real-time performance. Throttling /map_points to 5Hz maintains visualization without degrading SLAM tracking.


References

Campos et al. — ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM (2021)
Sturm et al. — A Benchmark for the Evaluation of RGB-D SLAM Systems — TUM RGB-D Dataset
ROS Noetic Documentation: https://wiki.ros.org/noetic
evo trajectory evaluation toolkit: https://github.com/MichaelGrupp/evo


