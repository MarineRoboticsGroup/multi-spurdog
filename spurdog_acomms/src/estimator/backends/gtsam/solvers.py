"""
GTSAM optimization solvers.

This module provides functions to solve factor graphs using different GTSAM optimizers.
"""

import numpy as np
import rospy  # type: ignore

from gtsam.gtsam import (
    NonlinearFactorGraph,
    Values,
    ISAM2,
    ISAM2Params,
    ISAM2DoglegParams,
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    GaussNewtonParams,
    GaussNewtonOptimizer,
    Symbol,
    Ordering,
)

from typing import Union, List


def diagnose_optimization_problem(
    graph: NonlinearFactorGraph, initial_vals: Values, verbose: bool = True
) -> dict:
    """
    Diagnose why GTSAM optimization might not be making progress.

    Performs comprehensive diagnostics including:
    - Sanity counts (factors, variables)
    - Key coverage and connectivity
    - Per-factor error analysis
    - Linearization and gradient analysis
    - Singular value decomposition

    Args:
        graph: The factor graph to diagnose
        initial_vals: Initial values for variables
        verbose: If True, log warnings with diagnostic info

    Returns:
        Dictionary with diagnostic results
    """
    diagnostics = {}

    # 0) Sanity counts
    num_factors = graph.size()
    num_vars = initial_vals.size()
    diagnostics["num_factors"] = num_factors
    diagnostics["num_vars"] = num_vars

    if verbose:
        rospy.logwarn(f"=== GTSAM Optimization Diagnostics ===")
        rospy.logwarn(f"#factors={num_factors}  #vars={num_vars}")

    # 1) Key coverage & connectivity suspects
    graph_keys = set()
    for i in range(graph.size()):
        try:
            for k in graph.at(i).keys():
                graph_keys.add(k)
        except Exception as e:
            if verbose:
                rospy.logerr(f"Cannot get keys from factor {i}: {e}")

    init_keys = set(initial_vals.keys())
    missing_in_initial = sorted(graph_keys - init_keys)
    unused_in_graph = sorted(init_keys - graph_keys)

    diagnostics["missing_in_initial"] = [
        Symbol(k).string() for k in missing_in_initial[:20]
    ]
    diagnostics["unused_in_graph"] = [Symbol(k).string() for k in unused_in_graph[:20]]

    if verbose:
        rospy.logwarn(f"Missing-in-initial: {diagnostics['missing_in_initial']}")
        rospy.logwarn(f"Unused-in-graph:   {diagnostics['unused_in_graph']}")

    # 2) Per-factor error @ initial (are they ~0? NaN? all ignored?)
    tot_error = 0.0
    zero_errors = 0
    nan_errors = 0
    error_list = []

    max_factors_to_check = min(graph.size(), 200)  # Avoid spam
    for i in range(max_factors_to_check):
        try:
            f = graph.at(i)
            e = f.error(initial_vals)
            error_list.append(e)

            if abs(e) < 1e-12:
                zero_errors += 1
            if np.isnan(e):
                nan_errors += 1
                if verbose:
                    rospy.logwarn(f"Factor {i} ({type(f).__name__}) has NaN error!")
            else:
                tot_error += e
        except Exception as ex:
            if verbose:
                rospy.logerr(f"factor[{i}] {type(f).__name__} error() threw: {ex}")
            error_list.append(None)

    diagnostics["total_error"] = tot_error
    diagnostics["zero_errors"] = zero_errors
    diagnostics["nan_errors"] = nan_errors
    diagnostics["error_list"] = error_list[:20]  # Keep first 20 for inspection

    if verbose:
        rospy.logwarn(
            f"Sum errors (initial) ≈ {tot_error:.6g}, zeros={zero_errors}, nans={nan_errors}"
        )
        if error_list:
            valid_errors = [e for e in error_list if e is not None and not np.isnan(e)]
            if valid_errors:
                rospy.logwarn(
                    f"Error stats: min={min(valid_errors):.3e}, max={max(valid_errors):.3e}, mean={np.mean(valid_errors):.3e}"
                )

    # 3) Linearization: is the gradient basically zero? Is the system rank-deficient?
    try:
        # Use standard Colamd ordering for NonlinearFactorGraph (simpler, no constraints needed)
        ordering = Ordering.ColamdNonlinearFactorGraph(graph)  # type: ignore

        linear = graph.linearize(initial_vals)
        A, b = linear.jacobian()  # Dense; fine for debugging scales/singularity

        # SVD analysis
        u, s, vt = np.linalg.svd(A, full_matrices=False)  # type: ignore
        min_singular = s[-1] if len(s) > 0 else 0.0
        max_singular = s[0] if len(s) > 0 else 0.0
        condition_number = (
            max_singular / min_singular if min_singular > 1e-15 else np.inf
        )

        diagnostics["min_singular_value"] = min_singular
        diagnostics["max_singular_value"] = max_singular
        diagnostics["condition_number"] = condition_number

        # Gradient norm: For least squares ||Ax - b||^2, gradient is A^T * (A*x - b)
        # At x=0 (or current estimate), gradient is approximately -A^T * b
        gradient = A.T.dot(b)  # type: ignore
        gradient_norm = np.linalg.norm(gradient)  # type: ignore
        diagnostics["gradient_norm"] = gradient_norm

        if verbose:
            rospy.logwarn(
                f"min singular value(A) = {min_singular:.3e}, max = {max_singular:.3e}"
            )
            rospy.logwarn(f"condition number = {condition_number:.3e}")
            rospy.logwarn(f"||gradient|| = {gradient_norm:.3e}")

            # Check for rank deficiency
            rank = np.sum(s > 1e-10 * max_singular)
            expected_rank = min(A.shape)
            diagnostics["matrix_rank"] = rank
            diagnostics["expected_rank"] = expected_rank

            if rank < expected_rank:
                rospy.logwarn(
                    f"⚠️  RANK DEFICIENT: rank={rank} < expected={expected_rank}"
                )

            # Check if gradient is essentially zero
            if gradient_norm < 1e-6:
                rospy.logwarn(
                    f"⚠️  GRADIENT NEAR ZERO: Already at local minimum or saddle point"
                )

    except Exception as ex:
        if verbose:
            rospy.logerr(f"Linearization analysis failed: {ex}")
        diagnostics["linearization_error"] = str(ex)

    # 4) Validate initial states (basic check)
    if verbose:
        rospy.logwarn(f"Initial values contain {initial_vals.size()} variables")

    if verbose:
        rospy.logwarn(f"=== End Diagnostics ===\n")

    return diagnostics


def try_gauss_newton_probe(
    graph: NonlinearFactorGraph, initial_vals: Values, verbose: bool = True
) -> Union[Values, None]:
    """
    Try Gauss-Newton optimization to see if it makes progress.

    If GN moves and reduces error, then LM's damping/scaling is likely the issue.

    Args:
        graph: The factor graph to optimize
        initial_vals: Initial values for variables
        verbose: If True, log diagnostic info

    Returns:
        Optimized Values if successful, None if failed
    """
    try:
        gnp = GaussNewtonParams()
        gnp.setMaxIterations(20)
        gnp.setVerbosity("TERMINATION")

        if verbose:
            rospy.logwarn("=== Trying Gauss-Newton as probe ===")

        gn = GaussNewtonOptimizer(graph, initial_vals, gnp)
        probe = gn.optimize()

        initial_error = graph.error(initial_vals)
        final_error = graph.error(probe)
        error_reduction = initial_error - final_error

        if verbose:
            rospy.logwarn(
                f"GN probe: initial_error={initial_error:.6f}, final_error={final_error:.6f}"
            )
            rospy.logwarn(f"GN probe: error_reduction={error_reduction:.6f}")

            if error_reduction > 0:
                rospy.logwarn(
                    "✓ Gauss-Newton reduced error → LM damping/scaling is the issue"
                )
            else:
                rospy.logwarn(
                    "✗ Gauss-Newton didn't help → Problem is deeper (rank deficiency, bad initial, etc.)"
                )

        return probe

    except Exception as ex:
        if verbose:
            rospy.logerr(f"Gauss-Newton probe failed: {ex}")
        return None


def solve_with_isam2(
    graph: NonlinearFactorGraph, initial_vals: Values, return_all_iterates: bool = False
) -> Union[Values, List[Values]]:
    """
    Solve a factor graph using ISAM2 (incremental smoothing and mapping).

    Args:
        graph: The factor graph to optimize.
        initial_vals: Initial values for all variables.
        return_all_iterates: If True, return all iterates (not supported for ISAM2).

    Returns:
        Optimized Values.

    Raises:
        NotImplementedError: If return_all_iterates is True.
    """
    if return_all_iterates:
        raise NotImplementedError("ISAM2 does not support returning all iterates")

    parameters = ISAM2Params()
    parameters.setOptimizationParams(ISAM2DoglegParams())
    rospy.logdebug(f"ISAM Params: {parameters}")
    isam_solver = ISAM2(parameters)
    isam_solver.update(graph, initial_vals)
    result = isam_solver.calculateEstimate()
    return result


def solve_with_levenberg_marquardt(
    graph: NonlinearFactorGraph,
    initial_vals: Values,
    return_all_iterates: bool = False,
    enable_diagnostics: bool = False,
    try_gauss_newton: bool = False,
) -> Union[Values, List[Values]]:
    """
    Solve a factor graph using Levenberg-Marquardt optimization.

    Args:
        graph: The factor graph to optimize.
        initial_vals: Initial values for all variables.
        return_all_iterates: If True, return all iterates (not currently supported).
        enable_diagnostics: If True, run diagnostic checks before optimization (default: False).
        try_gauss_newton: If True and LM fails, try Gauss-Newton as a probe (default: False).

    Returns:
        Optimized Values.

    Raises:
        ValueError: If variables in graph are not initialized or vice versa.
        RuntimeError: If optimization fails.
    """

    def _check_all_variables_have_initialization():
        """Verify that all graph variables are initialized and vice versa."""
        init_vals_vars = initial_vals.keys()
        graph_vars = graph.keyVector()

        graph_var_set = set(graph_vars)
        init_vals_var_set = set(init_vals_vars)

        in_graph_not_init_vals = graph_var_set - init_vals_var_set
        in_init_vals_not_graph = init_vals_var_set - graph_var_set

        if len(in_graph_not_init_vals) > 0:
            graph_vars_as_symbols = [Symbol(key) for key in in_graph_not_init_vals]
            raise ValueError(
                f"Variables in graph but not in initial values: {graph_vars_as_symbols}"
            )

        if len(in_init_vals_not_graph) > 0:
            init_vals_vars_as_symbols = [Symbol(key) for key in in_init_vals_not_graph]
            raise ValueError(
                f"Variables in initial values but not in graph: {init_vals_vars_as_symbols}"
            )

    _check_all_variables_have_initialization()

    # Run diagnostics before optimization
    if enable_diagnostics:
        rospy.logwarn("Running pre-optimization diagnostics...")
        diagnostics = diagnose_optimization_problem(graph, initial_vals, verbose=True)

        # Check for obvious problems
        if diagnostics.get("nan_errors", 0) > 0:
            rospy.logerr(
                f"⚠️  Found {diagnostics['nan_errors']} factors with NaN errors!"
            )

        if diagnostics.get("missing_in_initial"):
            rospy.logerr(
                f"⚠️  Missing variables in initial values: {diagnostics['missing_in_initial']}"
            )

        condition_num = diagnostics.get("condition_number", 0)
        if condition_num > 1e12:
            rospy.logwarn(
                f"⚠️  Very high condition number ({condition_num:.3e}) - system may be ill-conditioned"
            )

        run_full_diagnostics(graph, initial_vals)

    # Configure LM parameters with more relaxed settings
    params = LevenbergMarquardtParams()
    params.setMaxIterations(50)
    params.setRelativeErrorTol(0.0)  # Disable relative error tolerance
    params.setAbsoluteErrorTol(0.0)  # Disable absolute error tolerance
    params.setlambdaInitial(1e-6)  # Smaller starting λ (less aggressive damping)
    params.setlambdaFactor(5.0)  # Gentler growth of λ
    params.setVerbosity("TERMINATION")
    params.setVerbosityLM("LAMBDA")  # Extra verbose for LM iterations

    initial_error = graph.error(initial_vals)
    rospy.logwarn(f"Initial error: {initial_error:.6f}")

    try:
        rospy.logwarn("Running Levenberg-Marquardt optimization...")
        optimizer = LevenbergMarquardtOptimizer(graph, initial_vals, params)
        result = optimizer.optimize()

        final_error = graph.error(result)
        error_reduction = initial_error - final_error

        rospy.logwarn(
            f"Levenberg-Marquardt complete: "
            f"initial={initial_error:.6f}, final={final_error:.6f}, "
            f"reduction={error_reduction:.6f}"
        )

        # Check if optimization actually improved things
        if error_reduction < 1e-6:
            rospy.logwarn("⚠️  LM made minimal progress!")

            if try_gauss_newton:
                rospy.logwarn("Trying Gauss-Newton probe...")
                gn_result = try_gauss_newton_probe(graph, initial_vals, verbose=True)
                if gn_result is not None:
                    gn_error = graph.error(gn_result)
                    if gn_error < final_error:
                        rospy.logwarn(
                            f"✓ Gauss-Newton found better solution! Using GN result."
                        )
                        return gn_result

        return result

    except RuntimeError as e:
        rospy.logerr(f"Failed to run LevenbergMarquardtOptimizer: {e}")
        rospy.logerr(f"Graph keys: {graph.keyVector()}")
        rospy.logerr(f"Initial values: {initial_vals.keys()}")

        # Try Gauss-Newton as last resort
        if try_gauss_newton:
            rospy.logwarn("LM failed, trying Gauss-Newton as fallback...")
            gn_result = try_gauss_newton_probe(graph, initial_vals, verbose=True)
            if gn_result is not None:
                return gn_result

        raise e


def run_full_diagnostics(graph, initial_vals):
    """
    Run comprehensive diagnostics on a GTSAM optimization problem.

    Args:
        graph: GTSAM NonlinearFactorGraph
        initial_vals: GTSAM Values with initial estimates
    """
    rospy.logwarn("=" * 70)
    rospy.logwarn("GTSAM OPTIMIZATION FULL DIAGNOSTICS")
    rospy.logwarn("=" * 70)

    # 1. Run diagnostics
    diagnostics = diagnose_optimization_problem(graph, initial_vals, verbose=True)

    # 2. Check graph connectivity
    rospy.logwarn("\n" + "=" * 70)
    rospy.logwarn("CONNECTIVITY ANALYSIS")
    rospy.logwarn("=" * 70)

    # Build adjacency information
    variable_to_factors = {}  # Map from variable key to list of factor indices
    for i in range(graph.size()):
        try:
            factor = graph.at(i)
            factor_keys = factor.keys()
            for key in factor_keys:
                if key not in variable_to_factors:
                    variable_to_factors[key] = []
                variable_to_factors[key].append(i)
        except Exception as e:
            rospy.logwarn(f"Could not get keys from factor {i}: {e}")

    # Check if every variable has at least one factor
    all_variables = set(initial_vals.keys())
    variables_with_factors = set(variable_to_factors.keys())
    isolated_variables = all_variables - variables_with_factors

    if isolated_variables:
        rospy.logwarn(f"⚠️  Found {len(isolated_variables)} isolated variables (no factors attached):")
        isolated_symbols = [Symbol(k).string() for k in list(isolated_variables)[:10]]
        rospy.logwarn(f"   {isolated_symbols}")
        if len(isolated_variables) > 10:
            rospy.logwarn(f"   ... and {len(isolated_variables) - 10} more")
    else:
        rospy.logwarn(f"✓ All {len(all_variables)} variables have at least one factor")

    # Check graph connectivity using BFS
    if len(variables_with_factors) > 0:
        # Build adjacency graph: variables connected if they share a factor
        adjacency = {key: set() for key in variables_with_factors}
        for i in range(graph.size()):
            try:
                factor = graph.at(i)
                factor_keys = list(factor.keys())
                # Connect all pairs of variables in this factor
                for j, key1 in enumerate(factor_keys):
                    for key2 in factor_keys[j+1:]:
                        adjacency[key1].add(key2)
                        adjacency[key2].add(key1)
            except Exception:
                pass

        # BFS to find connected components
        visited = set()
        components = []
        
        for start_key in variables_with_factors:
            if start_key in visited:
                continue
                
            # New component found
            component = set()
            queue = [start_key]
            visited.add(start_key)
            component.add(start_key)
            
            while queue:
                current = queue.pop(0)
                for neighbor in adjacency.get(current, []):
                    if neighbor not in visited:
                        visited.add(neighbor)
                        component.add(neighbor)
                        queue.append(neighbor)
            
            components.append(component)
        
        num_components = len(components)
        diagnostics["num_connected_components"] = num_components
        diagnostics["component_sizes"] = [len(c) for c in components]
        
        if num_components == 1:
            rospy.logwarn(f"✓ Graph is fully connected ({len(variables_with_factors)} variables)")
        else:
            rospy.logwarn(f"❌ Graph has {num_components} disconnected components!")
            for idx, component in enumerate(components[:5]):  # Show first 5 components
                component_symbols = [Symbol(k).string() for k in list(component)[:5]]
                rospy.logwarn(f"   Component {idx+1}: {len(component)} variables, e.g., {component_symbols}")
            if num_components > 5:
                rospy.logwarn(f"   ... and {num_components - 5} more components")
    
    diagnostics["isolated_variables"] = isolated_variables
    diagnostics["variables_with_factors"] = variables_with_factors

    # 3. Print summary of issues found
    rospy.logwarn("\n" + "=" * 70)
    rospy.logwarn("DIAGNOSTIC SUMMARY")
    rospy.logwarn("=" * 70)

    issues_found = []

    if diagnostics.get("nan_errors", 0) > 0:
        issues_found.append(f"❌ {diagnostics['nan_errors']} factors have NaN errors")

    if diagnostics.get("missing_in_initial"):
        issues_found.append(
            f"❌ Variables missing in initial values: {len(diagnostics['missing_in_initial'])}"
        )

    if diagnostics.get("unused_in_graph"):
        issues_found.append(
            f"⚠️  Variables unused in graph: {len(diagnostics['unused_in_graph'])}"
        )

    condition_num = diagnostics.get("condition_number", 0)
    if condition_num > 1e12:
        issues_found.append(
            f"❌ Very high condition number: {condition_num:.3e} (ill-conditioned)"
        )
    elif condition_num > 1e8:
        issues_found.append(f"⚠️  High condition number: {condition_num:.3e}")

    gradient_norm = diagnostics.get("gradient_norm", float("inf"))
    if gradient_norm < 1e-6:
        issues_found.append(
            f"⚠️  Gradient near zero: {gradient_norm:.3e} (at local minimum?)"
        )

    zero_errors = diagnostics.get("zero_errors", 0)
    total_factors = diagnostics.get("num_factors", 1)
    if zero_errors > total_factors * 0.5:
        issues_found.append(
            f"⚠️  Many zero-error factors: {zero_errors}/{total_factors}"
        )

    matrix_rank = diagnostics.get("matrix_rank", 0)
    expected_rank = diagnostics.get("expected_rank", matrix_rank)
    if matrix_rank < expected_rank:
        issues_found.append(
            f"❌ Rank deficient: rank={matrix_rank} < expected={expected_rank}"
        )

    # Connectivity issues
    num_components = diagnostics.get("num_connected_components", 1)
    if num_components > 1:
        component_sizes = diagnostics.get("component_sizes", [])
        issues_found.append(
            f"❌ Graph disconnected: {num_components} components with sizes {component_sizes}"
        )
    
    isolated_vars = diagnostics.get("isolated_variables", set())
    if len(isolated_vars) > 0:
        issues_found.append(
            f"❌ Isolated variables (no factors): {len(isolated_vars)} variables"
        )

    if not issues_found:
        rospy.logwarn("✓ No obvious issues detected in diagnostics")
    else:
        rospy.logwarn(f"Found {len(issues_found)} potential issues:")
        for issue in issues_found:
            rospy.logwarn(f"  {issue}")

    # 3. Recommendations
    rospy.logwarn("\n" + "=" * 70)
    rospy.logwarn("RECOMMENDATIONS")
    rospy.logwarn("=" * 70)

    if diagnostics.get("nan_errors", 0) > 0:
        rospy.logwarn("• Check for NaN values in measurement data")
        rospy.logwarn("• Validate rotation representations (normalize quaternions)")
        rospy.logwarn("• Check for division by zero in factor implementations")

    if diagnostics.get("missing_in_initial"):
        rospy.logwarn("• Add initial values for all variables referenced in factors")

    if condition_num > 1e8:
        rospy.logwarn("• Check noise model scales (sigmas too large or too small?)")
        rospy.logwarn("• Ensure units are consistent (meters, radians, etc.)")
        rospy.logwarn("• Consider normalizing/scaling your problem")

    if matrix_rank < expected_rank:
        rospy.logwarn(
            "• Add gauge-fixing prior for pose(s) to anchor the coordinate frame"
        )
        rospy.logwarn("• Check if graph has multiple disconnected components")
        rospy.logwarn("• Ensure sufficient constraints (need 6 DOF constrained for 3D)")

    if gradient_norm < 1e-6:
        rospy.logwarn(
            "• Already at a stationary point - may need better initial values"
        )
        rospy.logwarn("• Or the problem is too weakly constrained")

    if num_components > 1:
        rospy.logwarn(
            f"• Graph has {num_components} disconnected components - each needs its own anchor/prior"
        )
        rospy.logwarn("• Add odometry or other factors to connect components")
        rospy.logwarn("• Consider if this is expected (e.g., multiple independent robots)")
    
    if len(isolated_vars) > 0:
        rospy.logwarn(
            f"• {len(isolated_vars)} variables have no factors attached"
        )
        rospy.logwarn("• Remove unused variables from initial_vals or add factors for them")

    # 4. Try Gauss-Newton probe
    rospy.logwarn("\n" + "=" * 70)
    rospy.logwarn("GAUSS-NEWTON PROBE")
    rospy.logwarn("=" * 70)

    gn_result = try_gauss_newton_probe(graph, initial_vals, verbose=True)

    if gn_result is not None:
        rospy.logwarn("✓ Gauss-Newton succeeded - consider using it instead of LM")
        rospy.logwarn("  or the issue is with LM's damping strategy")
    else:
        rospy.logwarn("✗ Gauss-Newton also failed - problem is fundamental")

    rospy.logwarn("\n" + "=" * 70)
    rospy.logwarn("END DIAGNOSTICS")
    rospy.logwarn("=" * 70 + "\n")

    return diagnostics
