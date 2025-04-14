import numpy as np
import rospy

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