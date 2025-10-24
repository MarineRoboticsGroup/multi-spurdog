"""
ESTIMATOR CODEBASE REORGANIZATION PLAN
=======================================

This document outlines the recommended reorganization of the spurdog_acomms estimator codebase.

CURRENT PROBLEMS:
-----------------
1. estimator_helpers.py is 1147 lines - a monolithic dumping ground for everything
2. ROS-specific code mixed with core estimation logic throughout
3. Excessive and inconsistent logging (rospy.logdebug/logwarn everywhere)
4. Backend-specific code (GTSAM, CORA) not properly abstracted
5. Temporary/debug code still present (TEST CODE sections, temporary priors)
6. No clear public API - users must import from internal modules
7. Poor separation of concerns - visualization logic in node files
8. Redundant error handling with nested try-except blocks

PROPOSED NEW STRUCTURE:
-----------------------

estimator/
├── __init__.py                 # Public API exports
├── estimator.py                # Abstract Estimator base class (KEEP AS IS - mostly good)
├── estimator_manager.py        # Core manager (SIMPLIFIED - ROS removed)
├── values.py                   # EstimatorValues (KEEP AS IS - good)
│
├── types/                      # All data types
│   ├── __init__.py
│   ├── key.py                  # Key, KeyPair classes
│   ├── covariance.py           # RelPoseCovar3, RelPoseCovar6
│   ├── measurements.py         # RangeMeasurement, OdometryMeasurement2D/3D, DepthMeasurement
│   ├── variables.py            # Pose2D, Pose3D, Point2D, Point3D
│   └── enums.py                # EstimatorMode enum
│
├── utils/                      # Pure utility functions
│   ├── __init__.py
│   ├── validation.py           # Validators: bound_validator, tuple_length_validator, etc.
│   ├── transformations.py      # get_theta_from_rotation_matrix, get_rotation_matrix_from_quat, etc.
│   ├── conversions.py          # convert_odom3_to_odom2, project_range_to_2d
│   └── precision.py            # get_measurement_precisions_from_covariance_matrix, etc.
│
├── backends/                   # Backend implementations
│   ├── __init__.py
│   ├── gtsam/
│   │   ├── __init__.py
│   │   ├── estimator.py        # GtsamEstimator class
│   │   ├── solvers.py          # solve_with_isam2, solve_with_levenberg_marquardt
│   │   ├── factors.py          # Factor construction helpers
│   │   └── conversions.py      # get_gtsam_symbol_from_key, get_pose2_from_matrix, etc.
│   └── cora/
│       ├── __init__.py
│       ├── estimator.py        # CoraEstimator class
│       └── conversions.py      # get_cora_symbol_from_key
│
├── ros/                        # ROS-specific code ONLY
│   ├── __init__.py
│   ├── ros_interface.py        # ROS message parsing and subscriptions
│   ├── ros_manager.py          # ROS-aware EstimatorManager wrapper
│   └── visualization.py        # RViz marker generation
│
└── custom_factors/             # Custom GTSAM factors (KEEP AS IS)
    ├── __init__.py
    ├── PoseToPointFactor.py
    ├── SESyncFactor2d.py
    └── SESyncFactor3d.py


FILE-BY-FILE CHANGES:
----------------------

1. estimator_helpers.py (1147 lines) → SPLIT INTO:
   - types/key.py (60 lines)
   - types/covariance.py (180 lines)
   - types/measurements.py (220 lines)
   - types/variables.py (160 lines)
   - types/enums.py (10 lines)
   - utils/validation.py (130 lines)
   - utils/transformations.py (200 lines)
   - utils/conversions.py (100 lines)
   - utils/precision.py (100 lines)

2. estimator_manager.py (387 lines) → SPLIT INTO:
   - estimator_manager.py (120 lines) - core logic only
   - ros/ros_manager.py (180 lines) - ROS subscribers and message handling
   - ros/ros_interface.py (100 lines) - message parsing functions

3. gtsam_estimator.py (628 lines) → SPLIT INTO:
   - backends/gtsam/estimator.py (300 lines) - core GtsamEstimator
   - backends/gtsam/solvers.py (150 lines) - optimization functions
   - backends/gtsam/factors.py (100 lines) - factor construction
   - backends/gtsam/conversions.py (80 lines) - type conversions

4. cora_estimator.py (269 lines) → SPLIT INTO:
   - backends/cora/estimator.py (230 lines) - core CoraEstimator
   - backends/cora/conversions.py (40 lines) - type conversions

5. estimator_node.py (260 lines) → SPLIT INTO:
   - estimator_node.py (120 lines) - clean ROS node
   - ros/visualization.py (140 lines) - marker creation logic


CLEAN PUBLIC API (__init__.py):
--------------------------------

# estimator/__init__.py
from .estimator import Estimator
from .estimator_manager import EstimatorManager
from .values import EstimatorValues

# types module
from .types.key import Key, KeyPair
from .types.covariance import RelPoseCovar3, RelPoseCovar6
from .types.measurements import (
    RangeMeasurement,
    OdometryMeasurement2D,
    OdometryMeasurement3D,
    DepthMeasurement,
)
from .types.variables import Pose2D, Pose3D, Point2D, Point3D
from .types.enums import EstimatorMode

# backends
from .backends.gtsam.estimator import GtsamEstimator
from .backends.cora.estimator import CoraEstimator

__all__ = [
    "Estimator",
    "EstimatorManager",
    "EstimatorValues",
    "Key",
    "KeyPair",
    "RelPoseCovar3",
    "RelPoseCovar6",
    "RangeMeasurement",
    "OdometryMeasurement2D",
    "OdometryMeasurement3D",
    "DepthMeasurement",
    "Pose2D",
    "Pose3D",
    "Point2D",
    "Point3D",
    "EstimatorMode",
    "GtsamEstimator",
    "CoraEstimator",
]


IMPORT CHANGES:
---------------

BEFORE:
  from estimator.estimator_helpers import (
      Key, RangeMeasurement, OdometryMeasurement, DepthMeasurement,
      EstimatorMode, RelPoseCovar6, Pose3D, Pose2D, Point2D, Point3D,
      get_theta_from_transformation_matrix, get_quat_from_rotation_matrix,
      ...  # 20+ imports
  )

AFTER:
  from estimator import (
      Key, RangeMeasurement, EstimatorMode, Pose3D, Pose2D, Point2D, Point3D
  )
  from estimator.utils.transformations import (
      get_theta_from_transformation_matrix,
      get_quat_from_rotation_matrix,
  )


CODE CLEANUP TASKS:
-------------------

1. Remove ALL test/debug code:
   - "#### TEST CODE ####" sections
   - "#### TEMPORARY CODE -- ADD PRIORS ####"
   - Commented-out code blocks in solve_with_levenberg_marquardt
   - self.print_difference_between_estimates(current_estimate_copy)

2. Reduce excessive logging:
   - Remove rospy.logdebug calls from hot paths (loops, frequent calls)
   - Keep only ERROR and WARN for exceptional cases
   - Add INFO for major state transitions only
   - Remove redundant "Adding X..." "X added" pairs

3. Simplify error handling:
   - Remove nested try-except blocks in add_odometry
   - Use consistent approach: log once, re-raise
   - Don't catch generic Exception - be specific

4. Address TODOs:
   - Remove temporary priors being added in _specific_initialize_point
   - Fix "TEMPORARY CODE" warnings
   - Address "Will be much slower" warnings for Python factors

5. Improve abstractions:
   - Create Protocol for PoseVariable (Pose2D/Pose3D)
   - Create Protocol for PointVariable (Point2D/Point3D)
   - Create Protocol for Measurement
   - Reduces Union[] types everywhere


MIGRATION STEPS:
----------------

1. Create new directory structure
2. Split estimator_helpers.py into focused modules
3. Move backend-specific code to backends/
4. Extract ROS code to ros/
5. Create clean __init__.py files
6. Update all imports throughout codebase
7. Remove temporary/debug code
8. Run tests to verify functionality
9. Update documentation


BENEFITS:
---------

1. Clarity: Each file has one clear purpose
2. Maintainability: Easy to find and modify code
3. Testability: Util functions can be tested independently
4. Reusability: Core logic separate from ROS allows other interfaces
5. Performance: Reduced import overhead with focused modules
6. Readability: ~200 line files vs 1147 line monolith
7. Type Safety: Better type hints with protocols
8. Extensibility: Easy to add new backends


BACKWARD COMPATIBILITY:
-----------------------

To maintain compatibility, keep a deprecated estimator_helpers.py that re-exports:

# estimator_helpers.py (deprecated)
import warnings
warnings.warn(
    "estimator_helpers is deprecated. Import from estimator or submodules instead.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export everything for compatibility
from .types.key import *
from .types.covariance import *
from .types.measurements import *
from .types.variables import *
from .types.enums import *
from .utils.validation import *
from .utils.transformations import *
from .utils.conversions import *
from .utils.precision import *
