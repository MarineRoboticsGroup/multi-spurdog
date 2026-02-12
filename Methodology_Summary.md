# Morrison's CORA Methodology - Implementation Summary

**Date**: February 12, 2026  
**Branch**: `liz-updated`  
**Reference**: Morrison Thesis (20250820_1130am_jp_thesis.pdf)

---

## 1. Morrison's Key Findings from Thesis

### CORA Implementation Details

Based on Morrison's thesis "Certifiably Correct Range-aided SLAM (CORA)":

1. **Random Initialization**
   - CORA uses **random pose and landmark initialization**
   - Quoted from thesis: "CORA can and often is solved using random initialization"
   - Quoted from thesis: "CORA uses random pose and landmark initialization"
   - This is repeated multiple times (lines 1999, 2624, 2754, 2934, 3116, 3119)

2. **No Local Frame Transformation**
   - **NO mention** of "local frame" transformations anywhere in the thesis
   - **NO mention** of coordinate frame transformations for CORA
   - CORA operates directly in world frame with the factor graph data

3. **Comparison with Other Methods**
   - **GTSAM (LM)**: Initialized using vehicle's EKF-based state estimate
   - **ISAM2**: Initialized using vehicle's EKF-based state estimate  
   - **CORA**: Uses random initialization (robust to poor initialization)

4. **Key Advantage**
   - "CORA is robust to random initialization" - this is its strength
   - Other methods (LM, ISAM2) are sensitive to initialization quality
   - CORA's convex relaxation approach finds globally optimal solutions regardless

### What "Random Initialization" Actually Means

**CLARIFICATION**: Morrison's "random initialization" refers to:
- Random initial **guesses** for pose positions and orientations
- Random initial **guesses** for landmark positions
- Robustness to poor/arbitrary starting values for optimization

**IT DOES NOT MEAN**:
- Ignoring coordinate frame alignment
- Optimizing in arbitrary coordinate systems
- No need for consistent world frame representation

---

## 2. Morrison's Direct Advice to Us

Morrison provided the following guidance during our collaboration:

### Fix #1: First Pose Prior
> "My hunch is that your first pose isn't initialized properly"

**Problem**: Solver doesn't place weight on first pose value unless you add a prior  
**Solution**: Add tight prior constraint on first pose

### Fix #2: Pin Initial Orientation
> "try to pin the initial orientation"

**Problem**: Range measurements don't constrain orientation (only odometry does)  
**Solution**: Use very tight orientation constraint (1e-8) on first pose prior

### Fix #3: Weaken Odometry Noise Models
> "try weakening your between factor noise models"

**Problem**: Overly confident noise models force solver to fit noise as signal  
**Solution**: Increase position variance 100x, orientation variance 10000x

### Fix #4: Relative Poses
> "It generates relative poses (relative to the landmarks)"

**Context**: CORA's solution is inherently relative to the landmark network geometry  
**NOT MEANT TO IMPLY**: Manual coordinate frame transformations needed

---

## 3. Proper Estimator Implementation

### Both Estimators Should:

1. **Operate in the Same World Frame**
   - Use actual world coordinates from `integrated_state` messages
   - Both GTSAM and CORA should optimize in the **same coordinate system**
   - This enables direct comparison of results

2. **Initialize First Pose Correctly**
   - **Position**: Use first `integrated_state` position (world coordinates)
   - **Orientation**: Use first `integrated_state` orientation (world coordinates)
   - **Prior**: Add tight constraint to anchor the optimization

3. **Use Appropriate Noise Models**
   - Position covariance: Scaled appropriately for uncertainty
   - Orientation covariance: Much tighter (range doesn't constrain rotation)

### GTSAM-Specific Implementation

```python
# First pose initialization
first_position = first_integrated_state.pose.pose.position  # World frame
first_orientation = first_integrated_state.pose.pose.orientation  # World frame

# Create first pose in world frame
if dimension == 3:
    init_pose = Pose3D(
        key=key0,
        position=(first_position.x, first_position.y, first_position.z),
        orientation=(first_orientation.x, first_orientation.y, 
                    first_orientation.z, first_orientation.w)
    )

# Add tight prior constraint
init_pose.marginal_covariance = np.diag([
    1e-6, 1e-6, 1e-6,  # Position (loose)
    1e-8, 1e-8, 1e-8   # Orientation (very tight)
])
```

### CORA-Specific Implementation

```python
# First pose initialization - SAME as GTSAM
first_position = first_integrated_state.pose.pose.position  # World frame
first_orientation = first_integrated_state.pose.pose.orientation  # World frame

# Create first pose in world frame (same as GTSAM)
if dimension == 3:
    init_pose = Pose3D(
        key=key0,
        position=(first_position.x, first_position.y, first_position.z),
        orientation=(first_orientation.x, first_orientation.y,
                    first_orientation.z, first_orientation.w)
    )

# Add tight prior constraint (same as GTSAM)
init_pose.marginal_covariance = np.diag([
    1e-6, 1e-6, 1e-6,  # Position
    1e-8, 1e-8, 1e-8   # Orientation (tight)
])

# Subsequent poses: Can use random initialization (CORA is robust to this)
# OR initialize from integrated_state (doesn't hurt, may speed convergence)
```

**Key Point**: The "random initialization" robustness applies to **subsequent poses and landmarks**, NOT to the world frame alignment of the first pose.

---

## 4. Current Implementation Status

### ✅ Completed Fixes

1. **First Pose at Origin** (Morrison Fix #1)
   - ✅ Removed GPS/world orientation initialization
   - ✅ Currently initializes at (0, 0, 0) with identity rotation
   - ⚠️ **ISSUE**: This creates arbitrary coordinate frames for each estimator

2. **Tight Orientation Prior** (Morrison Fix #2)
   - ✅ First pose orientation constrained to 1e-8 variance
   - ✅ Position has looser 1e-6 variance
   - ✅ Prevents rotation drift as Morrison recommended

3. **Relaxed Odometry Noise** (Morrison Fix #3)
   - ✅ Position variance scaled 100x
   - ✅ Orientation variance scaled 10000x
   - ✅ Applied to between-factors only

4. **Integrated State Initialization** (Morrison Fix #5)
   - ✅ All poses initialized from `integrated_state` messages
   - ✅ Provides absolute world-frame position/orientation
   - ✅ Successfully initializes all 289 poses

5. **Between-Factor Computation** (Morrison Fix #6)
   - ✅ Computed from consecutive `integrated_state` messages
   - ✅ Race condition fixed (checks both keys exist)
   - ✅ Clean execution, no errors

### ⚠️ Current Issues

1. **Coordinate Frame Misalignment**
   - **Problem**: Local frame transformation disabled (per Morrison's thesis)
   - **Result**: GTSAM and CORA optimize in different arbitrary frames
   - **Evidence from Latest Test**:
     ```
     GTSAM:    First pose = (0.00004, 0.00003, -0.00003)  ≈ Origin
     CORA:     First pose = (516.10, -164.27, 35.25)      ≈ Random frame
     Odometry: First pose = (74.46, -191.27, -1.04)       ≈ World frame
     ```

2. **Misinterpretation of Morrison's Advice**
   - We interpreted "relative poses" as needing local→world transformations
   - Morrison actually meant CORA's solution is relative to landmark geometry
   - Thesis shows NO coordinate transformations in his implementation

3. **Origin Initialization vs World Frame Initialization**
   - Currently: First pose at (0, 0, 0) for both estimators
   - Should be: First pose at actual world coordinates from `integrated_state`
   - Reason: Enable direct comparison and meaningful results

---

## 5. What Still Needs Investigation

### Critical: Coordinate Frame Alignment

**Question**: How do we initialize both estimators in the same world frame while respecting Morrison's "random initialization" approach?

**Answer (Based on Thesis)**:
- Morrison's "random initialization" applies to **optimization variables**
- It does NOT mean ignoring the actual coordinate system
- Both estimators should:
  1. Use first `integrated_state` position/orientation for first pose
  2. Add tight prior on first pose (anchors world frame)
  3. Initialize subsequent poses from `integrated_state` (or randomly for CORA)
  4. Let optimization refine all poses given the data

**Implementation Priority**: HIGH
- Without this, estimators produce incomparable results
- Cannot evaluate CORA vs GTSAM performance
- Cannot validate against odometry reference

### Testing: Validation Metrics

**Question**: How do we properly evaluate estimator performance?

**Metrics to Track**:
1. **Surface Position Error**: Distance between final optimized pose and GPS at surface
2. **Trajectory Shape**: Visual comparison with integrated odometry
3. **Consistency**: Do both estimators agree on relative geometry?
4. **Rotation Alignment**: Are orientations consistent with odometry?

**Current Status**: Can't evaluate properly due to coordinate frame mismatch

### Performance: CORA Optimization Speed

**Observation from Logs**:
```
[CORA DEBUG] getRandomInitialGuess:
  Expected variable size: 1303
  Stiefel manifold n (num frames): 289
```

**Questions**:
- Is CORA's batch optimization completing successfully?
- Are the optimized results reasonable (hard to tell with wrong coordinate frame)?
- Does CORA need more iterations or different solver parameters?

**Status**: Need to fix coordinate frame first, then evaluate

---

## 6. Recommended Next Steps

### Step 1: Implement World Frame Initialization (CRITICAL)

**File**: `src/multi-spurdog/spurdog_acomms/src/estimator/estimator.py`

**Change**:
```python
# CURRENT (WRONG):
if self.dimension == 3:
    init_pose = Pose3D(
        key=key1,
        position=(0.0, 0.0, 0.0),  # Origin - creates arbitrary frame
        orientation=(0.0, 0.0, 0.0, 1.0)
    )

# SHOULD BE:
if self.dimension == 3:
    # Use first integrated_state for world frame alignment
    first_position = first_integrated_state.pose.pose.position
    first_orientation = first_integrated_state.pose.pose.orientation
    
    init_pose = Pose3D(
        key=key1,
        position=(first_position.x, first_position.y, first_position.z),
        orientation=(first_orientation.x, first_orientation.y,
                    first_orientation.z, first_orientation.w)
    )
```

**This change**:
- ✅ Respects Morrison's methodology (still uses tight prior)
- ✅ Aligns both estimators to same world frame
- ✅ Enables proper comparison of results
- ✅ Maintains "random initialization" for subsequent poses (CORA robust to this)

### Step 2: Verify Coordinate Frame Alignment

**Test**:
1. Run pink_solo mission with world frame initialization
2. Check first poses match:
   - GTSAM first pose ≈ (74.46, -191.27, -1.04)
   - CORA first pose ≈ (74.46, -191.27, -1.04)
   - Odometry first pose = (74.46, -191.27, -1.04)

**Expected Result**: All three should start at same position

### Step 3: Validate Optimization Results

**Metrics**:
1. Plot trajectories - should show spiral shape matching odometry
2. Check surface position error - final pose should match GPS
3. Compare GTSAM vs CORA - should have similar geometry, different accuracy
4. Verify rotation alignment - orientations should be consistent

### Step 4: Performance Tuning (If Needed)

Only after coordinate frame is correct:
- Adjust noise models if needed
- Tune CORA solver parameters
- Optimize batch vs incremental performance

---

## 7. Key Takeaways

### Morrison's Actual Methodology

1. **CORA uses random initialization** = Robust to poor initial guesses for optimization variables
2. **NOT random coordinate frames** = Still needs consistent world frame for meaningful results
3. **No local frame transformations** = Operates directly on world frame factor graph
4. **Tight first pose prior** = Anchors the optimization in world coordinates

### Our Implementation Lesson

**Misunderstanding**: 
- We thought "random initialization" + "relative poses" meant we needed local→world transformations
- We thought optimizing at origin was Morrison's approach

**Reality from Thesis**:
- CORA is robust to random **variable values**, not random **coordinate systems**
- Both estimators need **same world frame** for comparison
- First pose should be at **actual world coordinates** with tight prior

### Path Forward

1. Initialize first pose to world frame position (from `integrated_state`)
2. Add tight prior to anchor optimization
3. Let subsequent poses initialize from `integrated_state` (or randomly for CORA)
4. Both estimators optimize in same world coordinates
5. Results become directly comparable

---

## 8. References

### Morrison's Thesis
- **File**: `/home/lizgajski2/catkin_ws/20250820_1130am_jp_thesis.pdf`
- **Key Sections**:
  - 3.1.4: Solving Range-aided SLAM using Convex Optimization
  - 3.1.5: Certifiable Optimization
  - 5.1.2: A Note About Vehicle Odometry
  - 6.2: Comparative Evaluation of CORA in Multi-agent Trials

### Implementation Documents
- `MORRISON_FIXES_V1_SUMMARY.md` - Initial fixes based on Morrison's advice
- `LOCAL_FRAME_IMPLEMENTATION.md` - Our (incorrect) local frame transformation approach
- `ALIGNMENT_SUMMARY.md` - Previous alignment investigations

### Git History
- **Branch**: `liz-updated`
- **Latest Commit**: "Test: Disable local frame per Morrison's thesis - both estimators now in arbitrary frames"
- **Status**: Confirmed coordinate frame mismatch, ready for world frame initialization fix

---

## Appendix: Test Results Comparison

### With Local Frame Transformation (Previous)
```
GTSAM:    First = (0.00008, 0.00004, 0.00005)    Last = (-67.53, -219.15, 31.94)
CORA:     First = (-113.52, -378.34, 42.19)      Last = (-300.85, -361.33, 43.74)
Odometry: First = (74.46, -191.27, -1.04)        Last = (71.99, -190.10, -0.05)

Issue: CORA not at origin despite transformation, wrong coordinates
```

### Without Local Frame Transformation (Current)
```
GTSAM:    First = (0.00004, 0.00003, -0.00003)   Last = (-99.90, -138.71, -15.11)
CORA:     First = (516.10, -164.27, 35.25)       Last = (516.51, -51.66, -14.08)
Odometry: First = (74.46, -191.27, -1.04)        Last = (71.99, -190.10, -0.05)

Issue: Each estimator in different arbitrary coordinate frame, no comparison possible
```

### Target (World Frame Initialization)
```
GTSAM:    First = (74.46, -191.27, -1.04)        Last ≈ (72, -190, -0.05)
CORA:     First = (74.46, -191.27, -1.04)        Last ≈ (72, -190, -0.05)
Odometry: First = (74.46, -191.27, -1.04)        Last = (71.99, -190.10, -0.05)

Expected: All in same world frame, comparable trajectories, surface GPS match
```

---

**Document Status**: Draft for implementation planning  
**Next Action**: Implement world frame initialization for both estimators  
**Expected Outcome**: Coordinate-aligned GTSAM and CORA results for proper comparison
