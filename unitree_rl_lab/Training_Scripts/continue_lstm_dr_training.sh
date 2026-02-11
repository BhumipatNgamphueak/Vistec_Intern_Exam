#!/bin/bash

# Continue training LSTM-DR from checkpoint model_7400.pt
# --load_run expects ONLY the run folder name (timestamp), not full path

python scripts/rsl_rl/train.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --num_envs 4096 \
  --max_iterations 25000 \
  --headless \
  --resume \
  --load_run 2026-02-06_23-20-45 \
  --checkpoint model_7400.pt

