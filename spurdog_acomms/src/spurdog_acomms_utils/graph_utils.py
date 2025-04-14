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