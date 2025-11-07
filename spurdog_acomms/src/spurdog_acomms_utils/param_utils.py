"""Small helper utilities for reading ROS params with guarded warnings.

Provides a safe wrapper around rospy.get_param that will search the current
namespace, fall back to search_param, and optionally warn if the parameter is
missing. Designed to be importable even in non-ROS unit test environments.
"""
from typing import Any

def get_namespace_param(name: str, default: Any = None, warn_if_missing: bool = False) -> Any:
    """Retrieve a ROS param with namespace-aware searching and optional warning.

    Behaviour:
    - If rospy is not importable (e.g., running outside ROS), return default.
    - If param exists exactly as provided, return it.
    - If param exists under the current namespace, return that value.
    - If rospy.search_param finds a parameter, return that value.
    - Otherwise return default and optionally warn.

    Args:
        name: parameter name to look up (can be relative or absolute).
        default: value to return if param not found.
        warn_if_missing: if True, log a warning when param not found.

    Returns:
        The parameter value or default.
    """
    try:
        import rospy
    except Exception:
        # rospy not available (unit tests / tooling). Fall back to default.
        if warn_if_missing and default is None:
            try:
                print(f"[param_utils] rospy unavailable; param '{name}' missing, using default")
            except Exception:
                pass
        return default

    # 1) exact match
    try:
        if rospy.has_param(name):
            return rospy.get_param(name)
    except Exception:
        # Defensive: rospy.has_param may raise if name malformed
        pass

    # 2) try with current namespace
    try:
        ns = rospy.get_namespace() or ""
        if ns:
            fq = ns.rstrip('/') + '/' + name.lstrip('/')
            if rospy.has_param(fq):
                return rospy.get_param(fq)
    except Exception:
        pass

    # 3) try search_param to find an occurrence anywhere on the param server
    try:
        found = rospy.search_param(name)
        if found:
            return rospy.get_param(found)
    except Exception:
        pass

    # Not found — warn if requested and return default
    if warn_if_missing:
        rospy.logwarn(f"Parameter '{name}' not found (ns='{rospy.get_namespace()}'); using default={default}")

    return default
