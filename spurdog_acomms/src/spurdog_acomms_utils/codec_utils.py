import numpy as np
import rospy

# fill out codec scale factors
CODEC_SCALE_FACTORS = {
    "init_prior": {
        "x": 10.0,
        "y": 10.0,
        "z": 100.0,
        "qx": 32767,
        "qy": 32767,
        "qz": 32767,
        "qw": 32767,
        "sigma_x": 300,
        "sigma_y": 300,
        "sigma_z": 300,
        "sigma_roll": 5000,
        "sigma_pitch": 5000,
        "sigma_yaw": 5000,
    },
    "partial_graph": {
        "x": 100.0,
        "y": 100.0,
        "z": 100.0,
        "qx": 127,
        "qy": 127,
        "qz": 127,
        "qw": 127,
        "sigma_x": 10,
        "sigma_y": 10,
        "sigma_z": 10,
        "sigma_roll": 10,
        "sigma_pitch": 10,
        "sigma_yaw": 10,
        "range": 100,
    }
}

# Pre-encoding and decoding functions:
def encode_init_prior_data_as_int(initial_position, initial_orientation, initial_sigmas):
    """This function encodes the initial prior factor data into a message
    Args:
        initial_position (np.array): The initial position
        initial_orientation (np.array): The initial orientation
        initial_sigmas (np.array): The initial sigmas
    Returns:
        PosePriorFactor: The encoded initial prior factor message
    """
    # Position is a int16[3], multiply by scale, then convert to int16
    x = int(initial_position[0] * CODEC_SCALE_FACTORS["init_prior"]["x"])
    y = int(initial_position[1] * CODEC_SCALE_FACTORS["init_prior"]["y"])
    z = int(initial_position[2] * CODEC_SCALE_FACTORS["init_prior"]["z"])
    # verify all fields are within int16 range
    if not (np.all(np.abs([x, y, z]) <= 32767)):
        rospy.logerr("[%s] Initial position values out of range!" % rospy.Time.now())
        return None
    else:
        initial_position = np.array([x, y, z], dtype=np.int16)

    # Orientation is a int16[4] (quaternion)
    qx = int(initial_orientation[0] * CODEC_SCALE_FACTORS["init_prior"]["qx"])
    qy = int(initial_orientation[1] * CODEC_SCALE_FACTORS["init_prior"]["qy"])
    qz = int(initial_orientation[2] * CODEC_SCALE_FACTORS["init_prior"]["qz"])
    qw = int(initial_orientation[3] * CODEC_SCALE_FACTORS["init_prior"]["qw"])
    # verify all fields are within int16 range
    if not (np.all(np.abs([qx, qy, qz, qw]) <= 32767)):
        rospy.logerr("[%s] Initial orientation values out of range!" % rospy.Time.now())
        return None
    else:
        initial_orientation = np.array([qx, qy, qz, qw], dtype=np.int16)

    # Sigmas is a int16[6], multiply by scale, then convert to int16
    sx = int(initial_sigmas[0] * CODEC_SCALE_FACTORS["init_prior"]["sigma_x"])
    sy = int(initial_sigmas[1] * CODEC_SCALE_FACTORS["init_prior"]["sigma_y"])
    sz = int(initial_sigmas[2] * CODEC_SCALE_FACTORS["init_prior"]["sigma_z"])
    sroll = int(initial_sigmas[3] * CODEC_SCALE_FACTORS["init_prior"]["sigma_roll"])
    spitch = int(initial_sigmas[4] * CODEC_SCALE_FACTORS["init_prior"]["sigma_pitch"])
    syaw = int(initial_sigmas[5] * CODEC_SCALE_FACTORS["init_prior"]["sigma_yaw"])
    # verify all fields are within int16 range
    if not (np.all(np.abs([sx, sy, sz, sroll, spitch, syaw]) <= 32767)):
        rospy.logerr("[%s] Initial sigmas values out of range!" % rospy.Time.now())
        return None
    else:
        initial_sigmas = np.array([sx, sy, sz, sroll, spitch, syaw], dtype=np.int16)
    return initial_position, initial_orientation, initial_sigmas

def decode_init_prior_data_from_int(initial_position, initial_orientation, initial_sigmas):
    # Apply the reverse of the encoding process to decode the data
    # Position is a int16[3], divide by scale to get original value
    initial_position = np.array([
        initial_position[0] / CODEC_SCALE_FACTORS["init_prior"]["x"],
        initial_position[1] / CODEC_SCALE_FACTORS["init_prior"]["y"],
        initial_position[2] / CODEC_SCALE_FACTORS["init_prior"]["z"]
    ])
    # Orientation is a int16[4] (quaternion), divide by scale to get original value
    initial_orientation = np.array([
        initial_orientation[0] / CODEC_SCALE_FACTORS["init_prior"]["qx"],
        initial_orientation[1] / CODEC_SCALE_FACTORS["init_prior"]["qy"],
        initial_orientation[2] / CODEC_SCALE_FACTORS["init_prior"]["qz"],
        initial_orientation[3] / CODEC_SCALE_FACTORS["init_prior"]["qw"]
    ])
    # Sigmas is a int16[6], divide by scale to get original value
    initial_sigmas = np.array([
        initial_sigmas[0] / CODEC_SCALE_FACTORS["init_prior"]["sigma_x"],
        initial_sigmas[1] / CODEC_SCALE_FACTORS["init_prior"]["sigma_y"],
        initial_sigmas[2] / CODEC_SCALE_FACTORS["init_prior"]["sigma_z"],
        initial_sigmas[3] / CODEC_SCALE_FACTORS["init_prior"]["sigma_roll"],
        initial_sigmas[4] / CODEC_SCALE_FACTORS["init_prior"]["sigma_pitch"],
        initial_sigmas[5] / CODEC_SCALE_FACTORS["init_prior"]["sigma_yaw"]
    ])
    return initial_position, initial_orientation, initial_sigmas

def encode_partial_graph_data_as_int(id, position, orientation, sigmas):
    """This function encodes the partial graph data into a message
    Args:
        position (np.array): The relative position
        orientation (np.array): The relative orientation
        sigmas (np.array): The relative sigmas
    Returns:
        PartialGraph: The encoded partial graph message
    """
    # Position is a int8[3], multiply by scale, then convert to int8
    x = int(position[0] * CODEC_SCALE_FACTORS["partial_graph"]["x"])
    y = int(position[1] * CODEC_SCALE_FACTORS["partial_graph"]["y"])
    z = int(position[2] * CODEC_SCALE_FACTORS["partial_graph"]["z"])
    # verify all fields are within int8 range
    if not (np.all(np.abs([x, y, z]) <= 32767)):
        rospy.logerr("[%s] Relative position values out of range!" % rospy.Time.now())
        return None
    else:
        position = list([x, y, z])

    # Orientation is a int8[4] (quaternion), multiply by scale, then convert to int8
    qx = int(orientation[0] * CODEC_SCALE_FACTORS["partial_graph"]["qx"])
    qy = int(orientation[1] * CODEC_SCALE_FACTORS["partial_graph"]["qy"])
    qz = int(orientation[2] * CODEC_SCALE_FACTORS["partial_graph"]["qz"])
    qw = int(orientation[3] * CODEC_SCALE_FACTORS["partial_graph"]["qw"])
    # verify all fields are within int8 range
    if not (np.all(np.abs([qx, qy, qz, qw]) <= 127)):
        rospy.logerr("[%s] Relative orientation values out of range!" % rospy.Time.now())
        return None
    else:
        orientation = list([qx, qy, qz, qw])

    # Sigmas is a int8[6], multiply by scale, then convert to int8
    sx = int(sigmas[0] * CODEC_SCALE_FACTORS["partial_graph"]["sigma_x"])
    sy = int(sigmas[1] * CODEC_SCALE_FACTORS["partial_graph"]["sigma_y"])
    sz = int(sigmas[2] * CODEC_SCALE_FACTORS["partial_graph"]["sigma_z"])
    sroll = int(sigmas[3] * CODEC_SCALE_FACTORS["partial_graph"]["sigma_roll"])
    spitch = int(sigmas[4] * CODEC_SCALE_FACTORS["partial_graph"]["sigma_pitch"])
    syaw = int(sigmas[5] * CODEC_SCALE_FACTORS["partial_graph"]["sigma_yaw"])
    # verify all fields are within int8 range
    if not (np.all(np.abs([sx, sy, sz, sroll, spitch, syaw]) <= 255)):
        rospy.logerr("[%s] Relative sigmas values out of range!" % rospy.Time.now())
        return None
    else:
        sigmas = list([sx, sy, sz, sroll, spitch, syaw])
    return position, orientation, sigmas

def decode_partial_graph_data_from_int(position, orientation, sigmas):
    """This function decodes the partial graph data from a message
    Args:
        position (list): The relative position
        orientation (list): The relative orientation
        sigmas (list): The relative sigmas
    Returns:
        tuple: (position, orientation, sigmas)
    """
    # Decode the data by applying the reverse of the encoding process
    # Position is a int16[3], divide by scale to get original value
    position = np.array([
        position[0] / CODEC_SCALE_FACTORS["partial_graph"]["x"],
        position[1] / CODEC_SCALE_FACTORS["partial_graph"]["y"],
        position[2] / CODEC_SCALE_FACTORS["partial_graph"]["z"]
    ])

    # Orientation is a int8[4], divide by scale to get original value
    orientation = np.array([
        orientation[0] / CODEC_SCALE_FACTORS["partial_graph"]["qx"],
        orientation[1] / CODEC_SCALE_FACTORS["partial_graph"]["qy"],
        orientation[2] / CODEC_SCALE_FACTORS["partial_graph"]["qz"],
        orientation[3] / CODEC_SCALE_FACTORS["partial_graph"]["qw"]
    ])

    # Sigmas is a int8[6], divide by scale to get original value
    sigmas = np.array([
        sigmas[0] / CODEC_SCALE_FACTORS["partial_graph"]["sigma_x"],
        sigmas[1] / CODEC_SCALE_FACTORS["partial_graph"]["sigma_y"],
        sigmas[2] / CODEC_SCALE_FACTORS["partial_graph"]["sigma_z"],
        sigmas[3] / CODEC_SCALE_FACTORS["partial_graph"]["sigma_roll"],
        sigmas[4] / CODEC_SCALE_FACTORS["partial_graph"]["sigma_pitch"],
        sigmas[5] / CODEC_SCALE_FACTORS["partial_graph"]["sigma_yaw"]
    ])

    return position, orientation, sigmas

# Message checking functions:
def check_partial_graph_for_msg_size(num_btwn, num_rng_from_us, num_rng_to_us):
    """This function checks the partial graph data, validating that the data fits within the defined
    message structure. This function is used to check the data before it is sent to the modem.
    Args:
        partial_graph_data (list): The partial graph data
    Returns:
        bool: True if the data is valid, False otherwise
    """
    # Check for supportability
    if num_btwn > 6:
        rospy.logerr("[%s] Too many BTWN messages (%s), max is 6" % rospy.Time.now(), num_btwn)
        return False
    elif num_rng_from_us > 4:
        rospy.logerr("[%s] Too many RNG messages from us (%s), max is 4" % rospy.Time.now(), num_rng_from_us)
        return False
    elif num_rng_to_us > 2:
        rospy.logerr("[%s] Too many RNG messages to us (%s), max is 2" % rospy.Time.now(), num_rng_to_us)
        return False
    else:
        rospy.loginfo("[%s] Partial graph data fits the message structure" % rospy.Time.now())
        return True

def check_partial_graph_for_feasibility(partial_graph_data, num_btwn, num_rng_from_us, num_rng_to_us):
    """This function checks the partial graph data, validating that the data fits within the defined
    message structure. This function is used to check the data before it is sent to the modem.
    Args:
        partial_graph_data (list): The partial graph data
    Returns:
        bool: True if the data is valid, False otherwise
    """
    num_rngs = num_rng_from_us + num_rng_to_us
    # Check for supportability
    if num_btwn < 1:
        rospy.logerr("[%s] No BTWN messages (%s), min is 1" % rospy.Time.now(), num_btwn)
        return False
    else:
        # Verify that the number of between fields is equal to the number of range fields
        if num_btwn != num_rngs:
            rospy.logerr("[%s] Number of BTWN messages (%s) does not match number of range messages (%s)" % (rospy.Time.now(), num_btwn, num_rngs))
            return False
        else:
            btwn_keys = [key for key in partial_graph_data.keys() if key.startswith("BTWN")]
            # Check that the keys are not duplicated
            if len(btwn_keys) != len(set(btwn_keys)):
                rospy.logerr("[%s] BTWN keys are duplicated: %s" % (rospy.Time.now(), btwn_keys))
                return False
            else:
                # Check that the keys are sequential
                btwn_indices = [int(key.split("_")[2]) for key in btwn_keys]
                btwn_indices.sort()
                # Check that the keys are sequential
                for i in range(len(btwn_indices) - 1):
                    if btwn_indices[i] + 1 != btwn_indices[i + 1]:
                        rospy.logerr("[%s] BTWN keys are not sequential: %s" % (rospy.Time.now(), btwn_keys))
                        return False

                # Check that the range key is present in btwn_keys
                for key in partial_graph_data.keys():
                    if key.startswith("RNG"):
                        if partial_graph_data[key]["key1"] not in btwn_keys:
                            rospy.logerr("[%s] Range key1 (%s) is not present in BTWN keys: %s" % (rospy.Time.now(), partial_graph_data[key]["key1"], btwn_keys))
                        elif partial_graph_data[key]["key2"] not in btwn_keys:
                            rospy.logerr("[%s] Range key2 (%s) is not present in BTWN keys: %s" % (rospy.Time.now(), partial_graph_data[key]["key2"], btwn_keys))
                            # TODO: Consider deleting the range key if it is missing
                            return False
                        else:
                            pass
    # If all checks pass, return True
    rospy.loginfo("[%s] Partial graph data is feasible" % rospy.Time.now())
    return True

def check_partial_graph_msg(partial_graph_data):
    """This function checks the partial graph data, validating that the data fits within the defined
    message structure. This function is used to check the data before it is sent to the modem.
    - It does not check that the message is full, just that the data will fit within the message.
    Args:
        partial_graph_data (list): The partial graph data
    Returns:
        bool: True if the data is valid, False otherwise
    """
    num_btwn = len([key for key in partial_graph_data.keys() if key.startswith("BTWN")])
    num_rng_from_us = len([key for key in partial_graph_data.keys() if key.startswith("RNG") and partial_graph_data[key].get("range") is not None])
    num_rng_to_us = len([key for key in partial_graph_data.keys() if key.startswith("RNG") and partial_graph_data[key].get("range") is None])
    size_check = check_partial_graph_for_msg_size(num_btwn, num_rng_from_us, num_rng_to_us)
    feasibility_check = check_partial_graph_for_feasibility(partial_graph_data, num_btwn, num_rng_from_us, num_rng_to_us)
    if size_check and feasibility_check:
        rospy.loginfo("[%s] Partial graph data is SAT to encode" % rospy.Time.now())
        return True
    elif size_check and not feasibility_check:
        rospy.logerr("[%s] Partial graph data UNSAT- right size, but failed feasibility" % rospy.Time.now())
        return False
    elif not size_check and feasibility_check:
        rospy.logerr("[%s] Partial graph data UNSAT- right feasibility, but failed size" % rospy.Time.now())
        return False
    else:
        rospy.logerr("[%s] Partial graph data UNSAT" % rospy.Time.now())
        return False

