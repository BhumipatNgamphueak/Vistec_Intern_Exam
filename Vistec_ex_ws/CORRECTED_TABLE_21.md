# Table 21: Computational Performance Comparison (CORRECTED)

## Original Table (INCORRECT)
| Configuration | Training Time (h) | Deploy Latency (ms) | RTF (Gazebo) |
|--------------|-------------------|---------------------|--------------|
| Implicit (Baseline) | 2.0 | 0.0 | **1.0** ❌ |
| Implicit+DR | 2.1 | 0.0 | **1.0** ❌ |
| LSTM (no DR) | 2.3 | 0.0 | **1.0** ❌ |
| LSTM+DR | 2.5 | 0.0 | **1.0** ❌ |
| MLP (no DR) | 2.2 | 0.0 | **1.0** ❌ |
| MLP+DR | 2.4 | 0.0 | **1.0** ❌ |

## Corrected Table (Based on Actual Data)
| Configuration | Training Time (h) | Deploy Latency (ms) | RTF (Gazebo) | RTF Std | RTF (Isaac) |
|--------------|-------------------|---------------------|--------------|---------|-------------|
| Implicit (Baseline) | 2.0 | 0.6 | **0.94** | 0.08 | 1.0 |
| Implicit+DR | 2.1 | 0.6 | **0.88** | 0.11 | 1.0 |
| LSTM (no DR) | 2.3 | 0.6 | ~0.92* | ~0.10* | 1.0 |
| LSTM+DR | 2.5 | 0.6 | **0.92** | 0.12 | 1.0 |
| MLP (no DR) | 2.2 | 0.6 | ~0.93* | ~0.09* | 1.0 |
| MLP+DR | 2.4 | 0.6 | **0.94** | 0.10 | 1.0 |

*Estimated from available policies (no DR variants not in Task 3 data)

## Key Corrections

### 1. RTF (Real-Time Factor) in Gazebo
**WRONG:** All policies show RTF = 1.0
**CORRECT:** RTF ranges from 0.88 to 0.94 with variability

**Measured Values (Task 3, 5 episodes each):**
- Implicit: 0.942 ± 0.080
- Implicit+DR: 0.880 ± 0.113 (worst RTF)
- LSTM+DR: 0.918 ± 0.116
- MLP+DR: 0.940 ± 0.096 (best RTF)

### 2. Deploy Latency
**Current:** 0.0 ms (rounded)
**ACTUAL:** ~0.57 ms (measured control loop latency)
**STATUS:** ✅ Acceptable rounding (negligible compared to 20ms control cycle)

### 3. RTF in Isaac
**MISSING FROM TABLE:** Isaac maintains constant RTF = 1.0 (simulator enforced)
**IMPACT:** This RTF mismatch (Isaac 1.0 vs Gazebo 0.88-0.94) is the **#1 contributor** to sim2sim transfer error

## Impact Analysis

### RTF Variability Consequences

**From mismatch_sources_summary.csv:**
- **Contribution to COM RMSE:** ~500mm (38% of total 1.3m error)
- **Root Cause:** Gazebo's variable RTF causes timing jitter and drift accumulation
- **Evidence:** Episode duration varies in Gazebo but constant in Isaac

### Why This Matters

```
Over 20-second episode:
- Expected steps: 1000 control cycles (50 Hz × 20s)
- With RTF = 0.92: Actual cycles = 920 (80 cycles missed)
- Per cycle: 20ms delay accumulates
- Total drift: 80 × 20ms = 1.6s timing error
- Spatial impact: ~500mm position drift
```

## Updated Caption

**Table 21: Computational performance comparison.**
Configuration | Training Time (h) | Deploy Latency (ms) | RTF (Gazebo) | RTF Std | RTF (Isaac)
Training-time integration adds 10–15% overhead with **near-zero deployment latency** (0.6ms).
However, **Gazebo's variable RTF (0.88–0.94) vs Isaac's constant RTF (1.0) introduces significant
timing mismatch**, contributing ~500mm (38%) to COM transfer error. Comparison to deployment-time
compensation: would add 102 μs (LSTM) or 25 μs (MLP) per 50 Hz cycle; our approach: 0.6ms.

## Recommendation

**Update the table to include:**
1. Actual RTF measurements from Gazebo
2. RTF standard deviation (shows variability)
3. Isaac RTF for comparison
4. Brief note about RTF mismatch impact

**Alternative:** If keeping RTF column simple, add a footnote:
> *Note: Gazebo RTF varies (mean 0.88-0.94, std 0.08-0.12) vs Isaac's constant 1.0,
> contributing ~500mm (38%) to transfer error (see Section X.X).*
