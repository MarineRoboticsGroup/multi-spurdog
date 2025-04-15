import numpy as np
import rospy

def configure_modem_addresses(num_agents, num_landmarks, local_address):
    """Configure the modem addresses for the agents and landmarks.

    Args:
        num_agents (int): The number of agents.
        num_landmarks (int): The number of landmarks.
        local_address (int): The local address of the agent.

    Returns:
        list: The modem addresses for the agents and landmarks.
    """
    modem_addresses = {}
    if num_agents == 1 and num_landmarks == 0:
        rospy.logerr("[%s] No addresses to ping" % rospy.Time.now())
        return None
    else:
        for i in range(num_agents):
            letter = chr(ord('A') + i)
            if i == local_address:
                modem_addresses[letter] = [i, 0]
            else:
                modem_addresses[letter] = [i]
        for j in range(num_landmarks):
            address = j + i + 1
            modem_addresses["L%d" % j] = [address,0]
    return modem_addresses

def get_config_from_modem_addresses(modem_addresses):
    # Exmaine the modem addresses to get the num_agents, num_landmarks, and the local address
    num_landmarks = sum(1 for key in modem_addresses if key.startswith("L"))
    num_agents = len(modem_addresses) - num_landmarks
    local_address = [key for key, value in modem_addresses.items() if len(value) == 2][0]
    return num_agents, num_landmarks, local_address

def configure_single_agent_cycle(modem_addresses, num_landmarks, local_address):
    """Configure the cycle targets for a single agent.

    Args:
        num_landmarks (int): The number of landmarks.
        local_address (int): The local address of the agent.

    Returns:
        list: The cycle targets for the agent and landmarks.
    """
    expected_slots = 2
    active_slots = [0,1]
    cycle_targets = {i: [[], []] for i in range(expected_slots)}
    if num_landmarks == 1:
        cycle_targets[0] = [modem_addresses["L0"][0], modem_addresses["L0"][0]]
        cycle_targets[1] = [modem_addresses["L0"][0], modem_addresses["L0"][0]]
    elif num_landmarks == 2:
        cycle_targets[0] = [modem_addresses["L0"][0], modem_addresses["L1"][0]]
        cycle_targets[1] = [modem_addresses["L0"][0], modem_addresses["L1"][0]]
    else:
        rospy.logerr("[%s] No addresses to ping" % rospy.Time.now())
    return cycle_targets

def configure_double_agent_cycle(modem_addresses, num_landmarks, local_address):
    """Configure the cycle targets for two agents.

    Args:
        num_landmarks (int): The number of landmarks.
        local_address (int): The local address of the agent.

    Returns:
        list: The cycle targets for the agents and landmarks.
    """
    expected_slots = 4
    active_slots = [local_address, local_address + 2]
    other_agent_address = 1 if local_address == 0 else 0
    cycle_targets = {i: [[], []] for i in range(expected_slots)}
    if num_landmarks == 0:
        cycle_targets[active_slots[0]] = [other_agent_address, []]
        cycle_targets[active_slots[1]] = [other_agent_address, []]
    elif num_landmarks == 1:
        cycle_targets[active_slots[0]] = [other_agent_address, modem_addresses["L0"][0]]
        cycle_targets[active_slots[1]] = [other_agent_address, modem_addresses["L0"][0]]
    elif num_landmarks == 2:
        cycle_targets[active_slots[0]] = [other_agent_address, modem_addresses["L0"][0]]
        cycle_targets[active_slots[1]] = [other_agent_address, modem_addresses["L1"][0]]
    else:
        rospy.logerr("[%s] Invalid number of landmarks" % rospy.Time.now())
    return cycle_targets

def configure_triple_agent_cycle(modem_addresses, num_landmarks, local_address):
    """Configure the cycle targets for three agents.

    Args:
        num_landmarks (int): The number of landmarks.
        local_address (int): The local address of the agent.

    Returns:
        list: The cycle targets for the agents and landmarks.
    """
    expected_slots = 6
    active_slots = [local_address, local_address + 3]
    other_agent_addresses = [i for i in range(3) if i != local_address]
    cycle_targets = {i: [[], []] for i in range(expected_slots)}
    if num_landmarks == 0:
        cycle_targets[active_slots[0]] = [other_agent_addresses[0], []]
        cycle_targets[active_slots[1]] = [other_agent_addresses[1], []]
    elif num_landmarks == 1:
        cycle_targets[active_slots[0]] = [other_agent_addresses[0], modem_addresses["L0"][0]]
        cycle_targets[active_slots[1]] = [other_agent_addresses[1], modem_addresses["L0"][0]]
    elif num_landmarks == 2:
        cycle_targets[active_slots[0]] = [other_agent_addresses[0], modem_addresses["L0"][0]]
        cycle_targets[active_slots[1]] = [other_agent_addresses[1], modem_addresses["L1"][0]]
    else:
        rospy.logerr("[%s] Invalid number of landmarks" % rospy.Time.now())
    return cycle_targets

def configure_cycle_targets(modem_addresses):
    """Configure the cycle targets for the agents and landmarks.

    Args:
        num_agents (int): The number of agents.
        num_landmarks (int): The number of landmarks.

    Returns:
        list: The cycle targets for the agents and landmarks.
    """
    cycle_targets = []
    num_agents, num_landmarks, local_address = get_config_from_modem_addresses(modem_addresses)
    # Setup the expectde number of slots
    if num_agents == 1:
        cycle_targets = configure_single_agent_cycle(modem_addresses, num_landmarks, local_address)
    elif num_agents == 2:
        cycle_targets = configure_double_agent_cycle(modem_addresses, num_landmarks, local_address)
    elif num_agents == 3:
        cycle_targets = configure_triple_agent_cycle(modem_addresses, num_landmarks, local_address)
    else:
        rospy.logerr("[%s] Invalid number of agents" % rospy.Time.now())
        return None
    return cycle_targets