"""Compatibility helpers for landmark parameter formats.

The workspace historically used two formats for each landmark entry:
- list: L0: [x, y, z]
- dict: L0: { src: N, pos: [x, y, z] }

This small helper normalizes access so callers can request src or pos
without caring about the representation.
"""
from typing import Any, Dict, List, Optional


def _normalize_entry(value: Any) -> Dict[str, Any]:
    """Return a canonical dict with keys 'src' and 'pos' for a landmark entry.

    Args:
        value: either a list (pos) or a dict with keys 'src' and/or 'pos'.

    Returns:
        {'src': Optional[int], 'pos': Optional[List[float]]}
    """
    result = {"src": None, "pos": None}
    if value is None:
        return result
    # If it's a dict with explicit src/pos
    if isinstance(value, dict):
        result["src"] = value.get("src")
        # accept either 'pos' or legacy coordinate list stored directly
        pos = value.get("pos", None)
        if pos is None:
            # maybe the dict itself contains coordinate-like values; fall back to raw
            result["pos"] = None
        else:
            result["pos"] = list(pos)
        return result

    # If it's a list/tuple, treat as position [x,y,z]
    if isinstance(value, (list, tuple)):
        v = list(value)
        # Some older configs sometimes encode [src, pos] (two-element)
        if len(v) == 2 and isinstance(v[0], int) and isinstance(v[1], (list, tuple)):
            result["src"] = int(v[0])
            result["pos"] = list(v[1])
            return result
        # Otherwise treat as plain position
        result["pos"] = v
        return result

    # Unknown format: leave raw
    return result


def get_landmark_pos(landmarks: Dict[str, Any], name: str) -> Optional[List[float]]:
    """Return the position list for landmark `name`, or None if unavailable."""
    if landmarks is None:
        return None
    val = landmarks.get(name)
    if val is None:
        return None
    return _normalize_entry(val).get("pos")


def get_landmark_src(landmarks: Dict[str, Any], name: str) -> Optional[int]:
    """Return the numeric src for landmark `name`, or None if unavailable."""
    if landmarks is None:
        return None
    val = landmarks.get(name)
    if val is None:
        return None
    return _normalize_entry(val).get("src")


def validate_landmarks(landmarks: Dict[str, Any]) -> None:
    """Validate the landmarks mapping.

    Raises ValueError if the mapping is malformed. Expected format:
      { 'L0': [x,y,z] } or { 'L0': { 'pos':[x,y,z], 'src': int } }

    This function does not depend on rospy so it can be used in tests.
    """
    if not isinstance(landmarks, dict):
        raise ValueError("landmarks must be a dict mapping landmark names to entries")
    for k, v in landmarks.items():
        # keys should look like L0, L1, etc.
        if not isinstance(k, str) or not k.startswith("L"):
            raise ValueError(f"invalid landmark key: {k}")
        # normalize and check pos
        entry = _normalize_entry(v)
        pos = entry.get("pos")
        if pos is None or not isinstance(pos, list) or len(pos) < 3:
            raise ValueError(f"landmark {k} has invalid pos: {pos}")
        # src, if present, must be an int >= 0
        src = entry.get("src")
        if src is not None:
            try:
                if int(src) < 0:
                    raise ValueError(f"landmark {k} has invalid src: {src}")
            except Exception:
                raise ValueError(f"landmark {k} has invalid src: {src}")
    return
