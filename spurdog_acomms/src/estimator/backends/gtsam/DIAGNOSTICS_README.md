# GTSAM Optimization Diagnostics

## Overview

This directory contains comprehensive diagnostic tools for debugging GTSAM optimization problems, particularly when Levenberg-Marquardt gives up without making progress.

## The Problem

When you see this output:
```
Warning:  Levenberg-Marquardt giving up because cannot decrease error with maximum lambda
converged
errorThreshold: 194174.554088 <? 0
absoluteDecrease: 0 <? 0
relativeDecrease: 0 <? 0
iterations: 0 >? 50
```

It means LM couldn't reduce the error even with maximum damping. This diagnostic suite helps identify why.

## Quick Start

### 1. Enable Diagnostics in Your Node

Diagnostics are **disabled by default** for performance. Enable them via parameters:

```python
from estimator.ros import ROSEstimatorManager

# Enable diagnostics when creating the manager
manager = ROSEstimatorManager(
    "actor_0",
    mode=EstimatorMode.GTSAM_LM,
    enable_diagnostics=True,        # Enable diagnostic output
    try_gauss_newton_fallback=True  # Try GN if LM fails
)
```

Or via ROS parameters in a launch file:
```xml
<node name="estimator_node" pkg="spurdog_acomms" type="estimator_node.py">
    <param name="enable_diagnostics" value="true"/>
    <param name="try_gauss_newton_fallback" value="true"/>
</node>
```

### 2. Enable Diagnostics Directly in Solver

For lower-level control:

```python
from estimator.backends.gtsam import solve_with_levenberg_marquardt

# Enable diagnostics explicitly (disabled by default)
result = solve_with_levenberg_marquardt(
    graph, 
    initial_vals,
    enable_diagnostics=True,  # Default: False
    try_gauss_newton=True     # Will try GN if LM fails
)
```

### 3. Run Standalone Diagnostics

For more detailed analysis:

```python
from estimator.backends.gtsam.diagnostics import run_full_diagnostics

# Run comprehensive diagnostics
diagnostics = run_full_diagnostics(graph, initial_vals)
```

### 4. Use Individual Diagnostic Functions

```python
from estimator.backends.gtsam import (
    diagnose_optimization_problem,
    try_gauss_newton_probe
)

# Detailed diagnostics
diag = diagnose_optimization_problem(graph, initial_vals, verbose=True)

# Try alternative solver
gn_result = try_gauss_newton_probe(graph, initial_vals, verbose=True)
```

## What Gets Checked

### 1. Sanity Counts
- Number of factors vs. number of variables
- Are there enough constraints?

### 2. Key Coverage & Connectivity
- Variables in graph but missing initial values ❌
- Variables with initial values but not used in graph ⚠️
- Disconnected graph components

### 3. Per-Factor Error Analysis
- Factors with NaN errors ❌
- Factors with zero errors (already satisfied)
- Distribution of error magnitudes
- Total initial error

### 4. Linearization Analysis
- **Singular Value Decomposition**: 
  - Min/max singular values
  - Condition number (>1e12 is very bad)
  - Rank deficiency detection
- **Gradient Analysis**:
  - Gradient norm (near zero means at local minimum)
  - System matrix properties

### 5. Alternative Solver Probe
- Try Gauss-Newton to see if LM damping is the issue
- If GN succeeds but LM fails → LM parameter tuning needed
- If both fail → fundamental problem (rank deficiency, bad initial, etc.)

## Common Issues & Fixes

### Issue: "Rank deficient: rank=X < expected=Y"
**Cause**: Under-constrained system
**Fixes**:
- Add a strong prior on at least one pose (gauge fixing)
- Ensure all components of the graph are connected
- For 3D: need at least 6 DOF constrained (3 position + 3 orientation)

Example fix:
```python
from gtsam import noiseModel, PriorFactorPose3

# Add strong prior on first pose
prior_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]))
graph.add(PriorFactorPose3(symbol_A0, initial_pose, prior_noise))
```

### Issue: "Very high condition number"
**Cause**: Ill-conditioned system (poor scaling)
**Fixes**:
- Check noise model scales (sigmas)
- Ensure consistent units (all meters, all radians)
- Avoid mixing very large and very small numbers

Example:
```python
# BAD: Huge range sigma makes factor nearly irrelevant
range_noise = noiseModel.Isotropic.Sigma(1, 1000.0)  # 1000m sigma!

# GOOD: Realistic sigma
range_noise = noiseModel.Isotropic.Sigma(1, 0.1)  # 10cm sigma
```

### Issue: "N factors have NaN errors"
**Cause**: Invalid values in initial estimates or measurements
**Fixes**:
- Check for NaN in measurement data
- Normalize quaternions: `q / np.linalg.norm(q)`
- Validate rotation matrices are valid SO(3)
- Check for division by zero in custom factors

### Issue: "Gradient near zero"
**Cause**: Already at local minimum or saddle point
**Fixes**:
- Provide better initial values
- Add more/better constraints
- Check if problem is actually solvable

### Issue: "Gauss-Newton succeeded but LM failed"
**Cause**: LM damping strategy issue
**Fixes**:
- Use GN instead: `solve_with_isam2()` uses Dogleg (GN-like)
- Tune LM parameters (already done in updated solver):
  ```python
  params.setlambdaInitial(1e-6)  # Smaller starting λ
  params.setlambdaFactor(5.0)     # Gentler growth
  ```

## Updated LM Parameters

The `solve_with_levenberg_marquardt` function now uses improved parameters:

```python
params = LevenbergMarquardtParams()
params.setMaxIterations(50)
params.setRelativeErrorTol(0.0)  # Disabled - don't stop early
params.setAbsoluteErrorTol(0.0)  # Disabled - don't stop early
params.setlambdaInitial(1e-6)    # Smaller λ (less aggressive damping)
params.setlambdaFactor(5.0)       # Gentler λ growth
params.setVerbosity("TERMINATION")
params.setVerbosityLM("TRYLAMBDA")  # Extra verbose
```

## Example Usage in estimator_node.py

Add diagnostics to your estimator node:

```python
from estimator.backends.gtsam.diagnostics import run_full_diagnostics

# In your update loop or when optimization fails:
if not success:
    rospy.logwarn("Optimization failed, running diagnostics...")
    diag = run_full_diagnostics(
        manager.estimator.graph,
        manager.estimator.current_estimate.all_values
    )
```

## Interpreting Output

### Good Output:
```
=== GTSAM Optimization Diagnostics ===
#factors=25  #vars=10
Missing-in-initial: []
Unused-in-graph:   []
Sum errors (initial) ≈ 15.3, zeros=0, nans=0
min singular value(A) = 1.234e-02, max = 4.567e+01
condition number = 3.704e+03
||gradient|| = 2.345e+01
✓ No obvious issues detected
```

### Bad Output:
```
#factors=25  #vars=10
Missing-in-initial: ['a1', 'a2']  ❌
Sum errors (initial) ≈ 1.234e+20, zeros=0, nans=5  ❌
min singular value(A) = 1.234e-15, max = 4.567e+01
condition number = 3.704e+16  ❌
⚠️  RANK DEFICIENT: rank=8 < expected=10  ❌
```

## Files

- `solvers.py` - Main solver with integrated diagnostics
- `diagnostics.py` - Standalone diagnostic tool with recommendations
- `DIAGNOSTICS_README.md` - This file

## References

- GTSAM Documentation: https://gtsam.org/
- Factor Graphs: https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf
