# Monocular Visual SLAM – Real-time Mapping & Autonomous Navigation #
A real-time monocular Visual SLAM system built using ORB-SLAM3 and ROS Noetic, capable of building a 3D sparse map of an environment and estimating camera trajectory from a single RGB camera — no GPS or depth sensor required. Evaluated on the TUM RGB-D benchmark dataset with quantitative trajectory accuracy metrics (ATE, RPE) and visualized in RViz.

# Overview
Visual SLAM is a core capability in autonomous robotics — enabling a robot to understand where it is and build a map of its environment using only camera input. This project implements a full monocular SLAM pipeline covering tracking, local mapping, loop closing, and ROS-based visualization.

# System Architecture
Single camera feed → ORB feature extraction & matching → Camera pose estimation → Sparse 3D map building → ROS topic publishing → RViz visualization

# Results
Evaluated on TUM RGB-D benchmark sequences:
SequenceATE RMSE (m)Tracking Successfr1/xyz0.009498.2%fr1/desk0.018396.7%fr1/room0.034194.1%

# Requirements

Ubuntu 20.04 + ROS Noetic
OpenCV 4.x, Eigen3, Pangolin
Python 3.8+ with evo, numpy, matplotlib


# Installation
bashgit clone https://github.com/UZ-SLAMLab/ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh && ./build.sh
chmod +x build_ros.sh && ./build_ros.sh

# Dataset
Download TUM fr1/xyz sequence from vision.in.tum.de/data/datasets/rgbd-dataset and run associate.py to sync RGB timestamps.

# Running
Standalone:
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml PATH_TO_DATASET

With ROS:
roscore
rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM3/Monocular/TUM1.yaml
rviz -d config/slam_visualization.rviz

Trajectory Evaluation
evo_ape tum groundtruth.txt KeyFrameTrajectory.txt -va --plot
evo_rpe tum groundtruth.txt KeyFrameTrajectory.txt -va --plot

Key Learnings

Monocular SLAM cannot recover absolute scale — only relative geometry is measurable from a single camera.
ORB-SLAM3 struggles in textureless environments; CLAHE preprocessing improves feature detection in low-contrast scenes.
Loop closing via DBoW2 place recognition is critical for correcting long-term drift accumulation.
Throttling ROS map point publishing to 5Hz maintains real-time SLAM performance without visualization overhead.

References

Campos et al. — ORB-SLAM3 (2021)
TUM RGB-D Dataset — Sturm et al.
evo toolkit — github.com/MichaelGrupp/evo


