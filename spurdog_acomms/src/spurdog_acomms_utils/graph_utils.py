import numpy as np
import rospy
import scipy.spatial.transform as spt

def check_partial_graph_for_completeness(partial_graph, modem_addresses):
    """
    Check if the partial graph represents a complete cycle. This is limited,
    and is meant to be called repeatedly to check if the graph is complete.
    It does not check the contents of the graph, only the number of entries.

    Args:
        partial_graph (dict): The partial graph.
        modem_addresses (dict): The modem addresses.

    Returns:
        bool: True if the graph is complete, False otherwise.
    """
    # Get the scenario parameters
    num_landmarks = sum(1 for key in modem_addresses if key.startswith("L"))
    num_agents = len(modem_addresses) - num_landmarks
    local_address = [key for key, value in modem_addresses.items() if len(value) == 2][0]
    # Get the expected number of entries
    expected_num_initiated_ranges = 2 if (num_agents == 1 and num_landmarks == 0) else 4
    expected_num_received_ranges = 2 if num_agents > 1 else 0
    expected_num_relative_poses = expected_num_initiated_ranges + expected_num_received_ranges
    # Check the number of recorded entries of each type
    num_btwn = len([key for key in partial_graph.keys() if key.startswith("BTWN")])
    num_rng_from_us = len([key for key in partial_graph.keys() if key.startswith("RNG") and partial_graph[key].get("range") is not None])
    num_rng_to_us = len([key for key in partial_graph.keys() if key.startswith("RNG") and partial_graph[key].get("range") is None])

    # Check that we've logged the expected number of measurements
    if num_btwn != expected_num_relative_poses:
        rospy.logerr("[%s] Expected %d BTWN entries, but found %d" % (rospy.Time.now(), expected_num_relative_poses, num_btwn))
        return False
    if num_rng_from_us != expected_num_initiated_ranges:
        rospy.logerr("[%s] Expected %d RNG entries, but found %d" % (rospy.Time.now(), expected_num_initiated_ranges, num_rng_from_us))
        return False
    if num_rng_to_us != expected_num_received_ranges:
        rospy.logerr("[%s] Expected %d RNG entries, but found %d" % (rospy.Time.now(), expected_num_received_ranges, num_rng_to_us))
        return False
    return True

def convert_partial_graph_btwn_to_relative_pose(bwtn_entry):
    """
    Convert a BTWN entry to a relative pose.
    Args:
        bwtn_entry (dict): The BTWN entry ("key1", "key2", "position", "orientation", "sigmas").
    Returns:
        relative_pose (dict): The relative pose.
    """
    translation = bwtn_entry["position"]
    rotation = spt.Rotation.from_quat(bwtn_entry["orientation"]).as_matrix()
    # Create the transformation matrix
    transformation = np.eye(4)
    transformation[0:3, 0:3] = rotation
    transformation[0:3, 3] = translation
    # Get covariance from sigmas
    sigmas = bwtn_entry["sigmas"]
    covariance = np.zeros((6, 6))
    covariance[0:3, 0:3] = np.diag(sigmas[0:3]**2)
    covariance[3:6, 3:6] = np.diag(sigmas[3:6]**2)
    return transformation, covariance

def compute_jacobian(T_total, T_i):
    """
    Compute the adjoint of a transformation matrix.
    Args:
        T (np.ndarray): The transformation matrix.
    Returns:
        np.ndarray: The adjoint matrix.
    """
    T = T_total @ np.linalg.inv(T_i)
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    # Compute the skew symmetric matrix of t
    t_skew = np.array([[0, -t[2], t[1]],
                       [t[2], 0, -t[0]],
                       [-t[1], t[0], 0]])
    # Compute the adjoint matrix
    J = np.zeros((6, 6))
    J[:3, :3] = R
    J[3:, :3] = t_skew @ R
    J[3:, 3:] = R
    return J

def marginalize_partial_graph(partial_graph):
    """
    This is designed to be called when the partial graph is complete,
    but needs to be due to failed comms.
    - It composes the relative pose measurements into a single relative pose
    - It removes the range measurements
    Args:
        partial_graph (dict): The partial graph.
    Returns:
        relative_pose (dict): The relative pose.
    """
    # Compose the relative pose measurements
    relative_transformations  = []
    relative_covariances = []
    for key in partial_graph.keys():
        if key.startswith("BTWN"):
            # Convert the BTWN entry to a relative pose
            transformation, covariance = convert_partial_graph_btwn_to_relative_pose(partial_graph[key])
            relative_transformations.append(transformation)
            relative_covariances.append(covariance)
        else:
            del partial_graph[key]
    # Now marginalize the relative transformations
    T_total = np.eye(4)
    P_total = np.zeros((6, 6))
    for i in range(len(relative_transformations)):
        T_i = relative_transformations[i]
        P_i = relative_covariances[i]
        # Compose the transformations
        T_total = T_total @ T_i
        # Compose the covariances
        J_i = compute_jacobian(T_total, T_i)
        P_total = J_i @ P_total @ J_i.T + P_i
    return T_total, P_total

def compose_two_transformations(T1, P1, T2, P2):
    """
    Compose two transformation matrices.
    Args:
        T1 (np.ndarray): The first transformation matrix.
        T2 (np.ndarray): The second transformation matrix.
    Returns:
        np.ndarray: The composed transformation matrix.
    """
    T12 = np.eye(4)
    P12 = np.eye((6,6))
    T12 = T12 @ T1
    J12 = compute_jacobian(T12, T1)
    P12 = J12 @ P1 @ J12.T + P2
    T12 = T12 @ T2
    J12 = compute_jacobian(T12, T2)
    P12 = J12 @ P12 @ J12.T + P2
    return T12, P12

def correct_first_relative_pose_for_failed_cycle(initial_relative_pose, relative_pose_from_failed_cycle):
    """
    Correct the first pose for a failed pose.
    Args:
        initial_relative_pose (dict): The initial relative pose.
            "key1", "key2", "position", "orientation", "sigmas"
        relative_pose_from_failed_cycle (dict): The relative pose from the failed cycle.
            "key1", "key2", "position", "orientation", "sigmas"
    Returns:
        dict: The corrected relative pose.
    """
    # Convert the initial relative pose to a transformation matrix
    T_initial = np.eye(4)
    T_initial[0:3, 0:3] = spt.Rotation.from_quat(initial_relative_pose["orientation"]).as_matrix()
    T_initial[0:3, 3] = initial_relative_pose["position"]
    # Convert the sigmas to a covariance matrix
    sigmas = initial_relative_pose["sigmas"]
    P_initial = np.zeros((6, 6))
    P_initial[0:3, 0:3] = np.diag(sigmas[0:3]**2)
    P_initial[3:6, 3:6] = np.diag(sigmas[3:6]**2)
    # Convert the relative pose from the failed cycle to a transformation matrix
    T_failed = np.eye(4)
    T_failed[0:3, 0:3] = spt.Rotation.from_quat(relative_pose_from_failed_cycle["orientation"]).as_matrix()
    T_failed[0:3, 3] = relative_pose_from_failed_cycle["position"]
    # Convert the sigmas to a covariance matrix
    sigmas = relative_pose_from_failed_cycle["sigmas"]
    P_failed = np.zeros((6, 6))
    P_failed[0:3, 0:3] = np.diag(sigmas[0:3]**2)
    P_failed[3:6, 3:6] = np.diag(sigmas[3:6]**2)
    # Compute the composed transformation matrix
    T_composed, P_composed = compose_two_transformations(T_initial, P_initial, T_failed, P_failed)
    # Split back into positoin, quaternion and sigmas
    position = T_composed[0:3, 3]
    quaternion = spt.Rotation.from_matrix(T_composed[0:3, 0:3]).as_quat()
    sigmas = np.zeros(6)
    sigmas[0:3] = np.sqrt(np.diag(P_composed[0:3, 0:3]))
    sigmas[3:6] = np.sqrt(np.diag(P_composed[3:6, 3:6]))
    return position, quaternion, sigmas