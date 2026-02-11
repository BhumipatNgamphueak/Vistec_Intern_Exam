#!/bin/bash
# Test all 6 trained models
# For MLP and LSTM: use 25000 iterations
# For Implicit: use latest available checkpoints

echo "========================================="
echo "Testing All 6 Go2 Velocity Configurations"
echo "========================================="

# 1. MLP + Custom DR (25000 iterations)
echo ""
echo "=== 1/6: MLP + Custom DR (model_24999.pt ~25000) ==="
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-Custom \
  --num_envs 32 \
  --load_run 2026-02-03_15-54-07_work_good \
  --checkpoint model_24999.pt \
  --video \
  --video_length 1000

# 2. MLP - No DR (25000 iterations)
echo ""
echo "=== 2/6: MLP - No DR (model_24999.pt ~25000) ==="
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-MLP-No-DR \
  --num_envs 32 \
  --load_run 2026-02-05_00-58-13 \
  --checkpoint model_24999.pt \
  --video \
  --video_length 1000

# 3. LSTM + DR (25000 iterations)
echo ""
echo "=== 3/6: LSTM + DR (model_25000.pt) ==="
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-LSTM-DR \
  --num_envs 32 \
  --load_run 2026-02-07_11-16-35 \
  --checkpoint model_25000.pt \
  --video \
  --video_length 1000

# 4. LSTM - No DR (25000 iterations)
echo ""
echo "=== 4/6: LSTM - No DR (model_25000.pt) ==="
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-LSTM-No-DR \
  --num_envs 32 \
  --load_run 2026-02-07_22-16-50 \
  --checkpoint model_25000.pt \
  --video \
  --video_length 1000

# 5. Implicit + DR (Latest: 14600 iterations)
echo ""
echo "=== 5/6: Implicit + DR (model_14600.pt - LATEST) ==="
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit-DR \
  --num_envs 32 \
  --load_run 2026-02-05_18-14-00 \
  --checkpoint model_14600.pt \
  --video \
  --video_length 1000

# 6. Implicit - No DR (Latest: 8100 iterations)
echo ""
echo "=== 6/6: Implicit - No DR (model_8100.pt - LATEST) ==="
python scripts/rsl_rl/play.py \
  --task Unitree-Go2-Velocity-Implicit \
  --num_envs 32 \
  --load_run 2026-02-05_16-27-42 \
  --checkpoint model_8100.pt \
  --video \
  --video_length 1000

echo ""
echo "========================================="
echo "All tests completed!"
echo "Videos saved to logs/rsl_rl/*/videos/play/"
echo "========================================="
