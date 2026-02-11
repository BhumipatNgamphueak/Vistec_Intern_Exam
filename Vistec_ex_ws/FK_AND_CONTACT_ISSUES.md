# FK and Contact Detection Issues - Investigation Report

**Date**: 2026-02-10
**Status**: 🚨 CRITICAL ISSUES FOUND

---

## 🔴 Critical Findings Summary

### 1. Isaac Foot Positions Are WRONG

**With URDF-based FK + Base Rotation:**

| Leg | Isaac Foot Height | Gazebo Foot Height | Status |
|-----|-------------------|-------------------|--------|
| FL  | **-82mm** (penetrating!) | 0.5-32mm (good) | ❌ WRONG |
| FR  | **+118-214mm** (floating!) | -5 to 22mm (good) | ❌ WRONG |
| RL  | **+181-258mm** (floating!) | -21 to 6mm (good) | ❌ WRONG |
| RR  | **+439-471mm** (WAY up!) | -12 to 10mm (good) | ❌ WRONG |

**Impact:**
- ❌ No stride detection (feet don't make proper contact)
- ❌ No contact metrics
- ❌ Invalid CoM oscillation (includes penetration)
- ❌ Comparison with Gazebo is meaningless

---

### 2. Both Isaac AND Gazebo Use Unusual Gait Strategy

**Joint Configuration During Walking:**

```
                Isaac       Gazebo      Expected (typical trot)
─────────────────────────────────────────────────────────────
Abad (lateral)  ±5°         ±7°         ±10-15°
Hip (forward)   +45-60°     +45-65°     ±20-30°
Knee (bend)     +3-5°       -77 to -93° -30 to -60°
```

**Key Observations:**
- Both simulators show hip angles of **+45-60°** (very extended forward)
- Gazebo knees at **-85°** (deeply bent, near joint limit -156° to -48°)
- Isaac knees at **+3°** (near zero - THIS IS SUSPICIOUS!)
- Limited joint motion (mostly hip moving, abad/knee semi-static)

**This is NOT a typical biological trot** - it's a learned controller strategy.

---

### 3. Joint Angle Inconsistency Between Simulators

| Joint | Isaac Range | Gazebo Range | Notes |
|-------|-------------|--------------|-------|
| FL Abad | -3° to +5° | -7° to +1° | Similar, reasonable |
| FL Hip  | -3° to +5° | +34° to +57° | ⚠️ HUGE difference! |
| FL Knee | +2° to +7° | -92° to -83° | ⚠️ OPPOSITE signs! |

**Hypothesis**: Isaac may be using **different joint conventions** or **has a calibration offset** issue.

---

## 🔍 Root Cause Analysis

### Possible Explanations

#### A. Joint Angle Sign Convention Difference ⭐ MOST LIKELY

**Evidence:**
- URDF specifies knee joint limit: `-2.7227 to -0.8378 rad` (NEGATIVE range!)
- Gazebo knee: -85° ✓ (within negative range)
- Isaac knee: +3° ❌ (POSITIVE, outside expected range)

**Theory**: Isaac might be recording knee angles with opposite sign, or using a different zero reference.

#### B. Joint Offset or "Default Pose" Issue

**Evidence:**
- Isaac shows very small joint angles (all near zero)
- But robot IS moving (0.5 m/s)
- Suggests joints might be recorded as **deltas from a default pose**, not absolute

#### C. FK Implementation Error

**Status**: ✅ RULED OUT
- URDF-based FK tested and verified
- FK gives correct results for Gazebo data
- Base rotation properly accounted for

#### D. Base Height Calibration Issue

**Status**: ⚠️ PARTIAL
- Isaac base: 333mm
- Gazebo base: 316mm
- Difference: 17mm (not enough to explain 100-400mm foot errors)

---

## 📊 Detailed FK Verification

### Test 1: Neutral Pose

```
Joint angles: abad=0°, hip=0°, knee=0°
Expected:     Foot Z = -426mm (body frame)
Actual (FK):  Foot Z = -426mm ✓ CORRECT
```

### Test 2: Isaac Typical Pose

```
Joint angles: abad=-1.4°, hip=4.7°, knee=2.2°
FK result:    Foot Z = -424mm (body frame)
World frame:  Base 333mm + (-424mm) = -91mm ❌ Below ground!
```

### Test 3: Gazebo Typical Pose

```
Joint angles: abad=-2.9°, hip=55.6°, knee=-84.7°
FK result:    Foot Z = -307mm (body frame)
World frame:  Base 316mm + (-307mm) = +9mm ✓ Touching ground!
```

---

## 🎯 Implications

### Why Isaac Shows No Stride Metrics

1. **No proper contact detection**: Feet at wrong heights
   - FL: Always penetrating ground by 82mm
   - FR/RL/RR: Floating 100-400mm above ground

2. **Cannot detect heel strikes**: Need foot to transition from air → ground

3. **Stride length = 0**: No horizontal foot displacement during swing phase

### Why Gazebo Works

1. **Proper ground contact**: All feet touch ground appropriately
2. **Contact transitions detected**: Can identify stance/swing phases
3. **Limited but valid strides**: Hip-driven forward motion creates small strides

---

## 🔧 Recommended Fixes

### Option 1: Investigate Isaac Joint Angle Convention ⭐ RECOMMENDED

**Action Items:**
1. Check Isaac Sim source code for joint angle conventions
2. Test if knee angles need sign flip: `knee_corrected = -knee_raw`
3. Check if there's a "default pose" offset that needs to be subtracted
4. Compare `joint_pos` vs `action` columns in CSV

**Test:**
```python
# Try negating knee angles for Isaac
isaac_knee_corrected = -isaac_knee_raw
# Re-run FK and check if feet touch ground
```

### Option 2: Calibrate Base Height Offset

**Action Items:**
1. Measure ground contact empirically from Isaac data
2. Calculate required base height offset
3. Apply offset: `base_z_corrected = base_z_raw + offset`

**Limitation**: Won't fix relative foot height differences between legs

### Option 3: Use Gazebo Data Only

**Pros:**
- Gazebo FK is working correctly
- Can get valid stride metrics
- Ground contact detected properly

**Cons:**
- Cannot compare Isaac vs Gazebo transfer
- Original research goal compromised

---

## 📈 Next Steps

### Immediate Actions

1. **Verify Joint Convention Hypothesis**:
   ```bash
   python investigate_isaac_joint_convention.py
   ```
   - Check `joint_pos` vs `action` relationship
   - Test sign flips for hip and knee
   - Validate against URDF joint limits

2. **Contact Isaac Sim Documentation**:
   - Check official docs for joint angle conventions
   - Look for known issues with GO2 robot model
   - Compare with Gazebo URDF interpretation

3. **Generate Corrected Analysis**:
   - Apply identified corrections
   - Re-run FK with corrected angles
   - Validate foot positions touch ground
   - Re-compute all stride metrics

### Long-term Solutions

1. **Standardize Data Format**:
   - Define canonical joint angle convention
   - Document zero-pose reference
   - Add validation checks to data collection

2. **Improve FK Implementation**:
   - Use `pinocchio` or `robotics-toolbox-python` for URDF parsing
   - Automatic handling of joint limits and conventions
   - Built-in collision detection

3. **Add Data Quality Checks**:
   - Validate foot heights are physically plausible
   - Check joint angles are within URDF limits
   - Warn if contact detection fails

---

## 📝 Code Changes Made

### Files Created:
1. `fk_from_urdf.py` - URDF-based forward kinematics
2. `analyze_contacts_and_stride.py` - Contact detection with proper FK
3. `FK_VERIFICATION_REPORT.md` - Initial FK analysis
4. `FK_AND_CONTACT_ISSUES.md` - This comprehensive report

### Files Modified:
1. `analyze_task.py` - Fixed Isaac episode mapping for tasks
2. `sim2sim_visualization.py` - Added extended CoM window support

---

## ⚠️ Current Status

**Isaac Data**: ❌ **NOT USABLE** for stride/contact analysis until joint convention issue resolved

**Gazebo Data**: ✅ **WORKING** - FK correct, contacts detected, stride metrics valid

**Comparison**: ❌ **BLOCKED** - Cannot compare until Isaac data fixed

---

## 📞 Questions for Investigation

1. **What is Isaac Sim's joint angle convention?**
   - Are angles absolute or relative to default pose?
   - What is the zero/neutral pose configuration?
   - Do knee angles use opposite sign from URDF?

2. **How does the controller output actions?**
   - Are `action` columns the controller output?
   - Are `joint_pos` the actual measured positions?
   - Is there a PD controller converting actions → positions?

3. **Why do both simulators show unusual gait?**
   - Is this the optimal learned strategy?
   - Would a different reward function produce biological gaits?
   - Is the crouched pose necessary for stability?

---

**Investigation ongoing. Will update as new findings emerge.**

**Last Updated**: 2026-02-10
