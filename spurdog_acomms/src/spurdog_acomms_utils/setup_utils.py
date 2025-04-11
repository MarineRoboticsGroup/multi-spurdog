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
    modem_addresses = []
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

def configure_cycle_targets(modem_addresses):
    """Configure the cycle targets for the agents and landmarks.

    Args:
        num_agents (int): The number of agents.
        num_landmarks (int): The number of landmarks.

    Returns:
        list: The cycle targets for the agents and landmarks.
    """
    cycle_targets = []
    # Exmaine the modem addresses to get the num_agents, num_landmarks, and the local address
    num_landmarks = sum(1 for key in modem_addresses if key.startswith("L"))
    num_agents = len(modem_addresses) - num_landmarks
    local_address = [key for key, value in modem_addresses.items() if len(value) == 2][0]

    return cycle_targets