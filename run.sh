#!/bin/bash
# ── Monocular Visual SLAM — One-click runner ──────────────────────────────────
# Usage: bash run.sh /path/to/tum_dataset

DATASET=$1
OUTPUT="results"

if [ -z "$DATASET" ]; then
  echo "Usage: bash run.sh /path/to/rgbd_dataset_freiburg1_xyz"
  exit 1
fi

echo "=============================="
echo "  Monocular Visual SLAM"
echo "=============================="

# Step 1 — Run SLAM
echo -e "\n[1/3] Running SLAM pipeline..."
python3 slam_pipeline.py "$DATASET" --output "$OUTPUT"

# Step 2 — Evaluate
echo -e "\n[2/3] Evaluating trajectory..."
python3 evaluate_trajectory.py \
  --gt "$DATASET/groundtruth.txt" \
  --est "$OUTPUT/trajectory.txt" \
  --output "$OUTPUT/metrics.csv"

# Step 3 — Visualize
echo -e "\n[3/3] Generating plots..."
python3 visualize_results.py \
  --gt "$DATASET/groundtruth.txt" \
  --est "$OUTPUT/trajectory.txt" \
  --output "$OUTPUT"

echo -e "\nDone. Results saved in ./$OUTPUT/"
