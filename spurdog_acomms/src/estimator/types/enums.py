"""
Enumerations for estimator modes and types.
"""
from enum import Enum


class EstimatorMode(Enum):
    """Estimation backend modes."""
    GTSAM_LM = 1
    GTSAM_DOG = 2
    CORA = 3
