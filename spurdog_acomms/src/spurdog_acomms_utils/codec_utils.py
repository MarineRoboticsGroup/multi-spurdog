import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
from spurdog_acomms.msg import GraphUpdate, BetweenFactor, RangeFactor
from ros_acomms_msgs.msg import PingReply
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
    },
    "prior_factor": {
        "x": 10.0,              # int16, -32768 to 32767, expected: -3200m to 3200m
        "y": 10.0,              # int16, -32768 to 32767, expected: -3200m to 3200m
        "z": 10.0,              # int8, -128 to 127, expected: -12.8 to 12.7m, resolution: 0.1m
        "qw": 127,              # int8, -128 to 127, expected: -1 to 1, resolution: 0.00787deg
        "qx": 127,              # int8, -128 to 127, expected: -1 to 1, resolution: 0.00787deg
        "qy": 127,              # int8, -128 to 127, expected: -1 to 1, resolution: 0.00787deg
        "qz": 127,              # int8, -128 to 127, expected: -1 to 1, resolution: 0.00787deg
        "sigma_x": 5,           # uint8, 0-255, expected: 0-50m, resolution: 0.2m
        "sigma_y": 5,           # uint8, 0-255, expected: 0-50m, resolution: 0.2m
        "sigma_z": 5,           # uint8, 0-255, expected: 0-50m, resolution: 0.2m
        "sigma_psi": 255e2,     # uint8, 0-255, expected: 1e-2
        "rho_xy": 0,            # int8, -128 to 127, expected: -1e-10 to 1e-10, resolution:
        "rho_xpsi": 0,          # int8, -128 to 127, expected: -5e-6 to 5e-6, resolution:
        "rho_ypsi": 0           # int8, -128 to 127, expected: -5e-6 to 5e-6, resolution:
    },
    "between_factor": {
        "x": 500.0,              # int16, -32768 to 32767, expected: -65.5 to 65.5m, resolution: 0.002m
        "y": 500.0,              # int16, -32768 to 32767, expected: -65.5 to 65.5m, resolution: 0.002m
        "z": 10.0,              # int8, -128 to 127, expected: -12.8 to 12.7m, resolution: 0.1m
        "qw": 127,              # int8, -128 to 127, expected: -1 to 1, resolution: 0.00787deg
        "qx": 127,              # int8, -128 to 127, expected: -1 to 1, resolution: 0.00787deg
        "qy": 127,              # int8, -128 to 127, expected: -1 to 1, resolution: 0.00787deg
        "qz": 127,              # int8, -128 to 127, expected: -1 to 1, resolution: 0.00787deg
        "sigma_x": 50,          # uint8, 0-255, expected: 0-5m, resolution: 0.02m
        "sigma_y": 50,          # uint8, 0-255, expected: 0-5m, resolution: 0.02m
        "sigma_z": 50,          # uint8, 0-255, expected: 0-5m, resolution: 0.02m
        "sigma_psi": 255e4,     # uint8, 0-255, expected: 1e-5 to 1e-4, resolution: 4e-7rad (2.2e-5 deg)
        "rho_xy": 127e12,       # int8, -128 to 127, expected: -1e-10 to 1e-10, resolution:
        "rho_xpsi": 254e5,      # int8, -128 to 127, expected: -5e-6 to 5e-6, resolution:
        "rho_ypsi": 254e5       # int8, -128 to 127, expected: -5e-6 to 5e-6, resolution:
    },
    "range_factor": {
        "remote_address": 1,    # uint8, 0-255, expected: 0-5
        "index": 1,             # uint8, 0-65535, integer value
        "measured_range": 50,   # uint16, 0-65535, expected: 0-1300m, resolution: 0.02m
        "sigma_range": 10,      # uint8, 0-255, expected: 0-25.5m, resolution: 0.1m
        "depth": 20,            # uint8, 0-255, expected: 0-12.5m, resolution: 0.05m
    }
}

# Graph Update encoding:
def encode_pwcs_as_int(pwcs:PoseWithCovarianceStamped, type):
    # Use type to get the scale factors
    if type == "between":
        codec_scale_factors = CODEC_SCALE_FACTORS["between_factor"]
    elif type == "prior":
        codec_scale_factors = CODEC_SCALE_FACTORS["prior_factor"]
    else:
        rospy.logerr("[%s] Invalid type for encoding PoseWithCovarianceStamped!" % rospy.Time.now())
        return BetweenFactor()
    # Intialize the output message
    between_factor = BetweenFactor()
    # Extract the appropriate covariance sigmas
    sigma_x = np.sqrt(pwcs.pose.covariance[0])  # x
    sigma_y = np.sqrt(pwcs.pose.covariance[7])  # y
    sigma_z = np.sqrt(pwcs.pose.covariance[14])  # z
    sigma_psi = np.sqrt(pwcs.pose.covariance[35])  # yaw
    # Extract the correlation coefficients for sigma_xy, sigma_xpsi, sigma_ypsi
    rho_xy = pwcs.pose.covariance[1] / (sigma_x * sigma_y) if sigma_x and sigma_y else 0.0
    rho_xpsi = pwcs.pose.covariance[5] / (sigma_x * sigma_psi) if sigma_x and sigma_psi else 0.0
    rho_ypsi = pwcs.pose.covariance[11] / (sigma_y * sigma_psi) if sigma_y and sigma_psi else 0.0

    # Encode the position and orientation
    x = int(np.clip(int(pwcs.pose.pose.position.x * codec_scale_factors["x"]), -32768, 32767))
    y = int(np.clip(int(pwcs.pose.pose.position.y * codec_scale_factors["y"]), -32768, 32767))
    z = int(np.clip(int(pwcs.pose.pose.position.z * codec_scale_factors["z"]),  -128, 127))

    # Encode the orientation as a quaternion
    qx = int(np.clip(int(pwcs.pose.pose.orientation.x * codec_scale_factors["qx"]), -128, 127))
    qy = int(np.clip(int(pwcs.pose.pose.orientation.y * codec_scale_factors["qy"]), -128, 127))
    qz = int(np.clip(int(pwcs.pose.pose.orientation.z * codec_scale_factors["qz"]), -128, 127))
    qw = int(np.clip(int(pwcs.pose.pose.orientation.w * codec_scale_factors["qw"]), -128, 127))

    # Encode the sigmas
    sigma_x = int(np.clip(int(sigma_x * codec_scale_factors["sigma_x"]), 0, 255))
    sigma_y = int(np.clip(int(sigma_y * codec_scale_factors["sigma_y"]), 0, 255))
    sigma_z = int(np.clip(int(sigma_z * codec_scale_factors["sigma_z"]), 0, 255))
    sigma_psi = int(np.clip(int(sigma_psi * codec_scale_factors["sigma_psi"]), 0, 255))

    # Encode the correlation coefficients
    rho_xy = int(np.clip(int(rho_xy * codec_scale_factors["rho_xy"]), -128, 127))
    rho_xpsi = int(np.clip(int(rho_xpsi * codec_scale_factors["rho_xpsi"]), -128, 127))
    rho_ypsi = int(np.clip(int(rho_ypsi * codec_scale_factors["rho_ypsi"]), -128, 127))

    # Build the list 
    between_factor = [x, y, z, qw, qx, qy, qz,
                      sigma_x, sigma_y, sigma_z, sigma_psi,
                      rho_xy, rho_xpsi, rho_ypsi]
    return between_factor

def encode_range_event_as_int(remote_address:int, remote_index:int=None, measured_range:float=None, sigma_range:float=None, depth:float=None):
    """This function encodes the range event data into a message
    Args:
        remote_address (int): The remote address of the range event
        remote_index (int): The index of the range event, defaults to None
        measured_range (float): The measured range value, defaults to None
        sigma_range (float): The sigma value for the range, defaults to None
        depth (float): The depth value, defaults to None
    Returns:
        RangeFactor: The encoded range factor message
    """
    # Encode the remote address
    remote_address = remote_address
    # Encode the index or measured range
    if remote_index is not None:
        remote_index = int(remote_index*CODEC_SCALE_FACTORS["range_factor"]["index"])
        if remote_index < 0 or remote_index > 65535:
            rospy.logerr("[%s] Index value out of range!" % rospy.Time.now())
            index_or_measured_range = 0  # Default value if out of range
        else:
            index_or_measured_range = remote_index
        sigma_range = 0  # Default value if index is provided
    elif measured_range is not None:
        measured_range = int(measured_range * CODEC_SCALE_FACTORS["range_factor"]["measured_range"])
        sigma_range = int(sigma_range * CODEC_SCALE_FACTORS["range_factor"]["sigma_range"])
        # Handle unexpected large sigmas
        if sigma_range < 0 or sigma_range > 255:
            rospy.logerr("[%s] Sigma range value out of range!" % rospy.Time.now())
            sigma_range = 255  # Default value if out of range
        # Handle unexpected low ranges by setting the range to zero with max sigma
        if measured_range < 0:
            rospy.logerr("[%s] Measured range value low out of range!" % rospy.Time.now())
            index_or_measured_range = 0  # Default value if out of range
            sigma_range = 255  # Default value if out of range
        # Handle unexpected high ranges by setting the range to max with max sigma
        elif measured_range > 65535:
            rospy.logerr("[%s] Measured range value high out of range!" % rospy.Time.now())
            index_or_measured_range = 65535  # Default value if out of range
            sigma_range = 255  # Default value if out of range
        # Otherwise, set the measured range and sigma range
        else:
            index_or_measured_range = measured_range
            sigma_range = sigma_range
    else:
        rospy.logerr("[%s] No index or measured range provided!" % rospy.Time.now())
        return RangeFactor()
    if depth is None or depth < 0.1:
        depth = 0.1
    elif depth > 12.5:
        depth = 12.5
    else:
        pass # Scale the depth
    depth = int(depth * CODEC_SCALE_FACTORS["range_factor"]["depth"])
    return [remote_address, index_or_measured_range, sigma_range, depth]

# def encode_range_event_as_int(remote_address:int, index:int=None, measured_range:float=None, sigma_range:float=None, depth:float=None):
#     """This function encodes the range event data into a message
#     Args:
#         remote_address (int): The remote address of the range event
#         index (int): The index of the range event, defaults to None
#         measured_range (float): The measured range value, defaults to None
#         sigma_range (float): The sigma value for the range, defaults to None
#         depth (float): The depth value, defaults to None
#     Returns:
#         RangeFactor: The encoded range factor message
#     """
#     # Initialize the output message
#     range_factor = RangeFactor()
#     # Encode the remote address
#     range_factor.remote_address = remote_address
#     # Encode the index or measured range
#     if index is not None:
#         index = int(index*CODEC_SCALE_FACTORS["range_factor"]["index"])
#         if index < 0 or index > 65535:
#             rospy.logerr("[%s] Index value out of range!" % rospy.Time.now())
#             range_factor.index_or_measured_range = 0  # Default value if out of range
#         else:
#             range_factor.index_or_measured_range = index
#         range_factor.sigma_range = 0  # Default value if index is provided
#     elif measured_range is not None:
#         measured_range = int(measured_range * CODEC_SCALE_FACTORS["range_factor"]["measured_range"])
#         sigma_range = int(sigma_range * CODEC_SCALE_FACTORS["range_factor"]["sigma_range"])
#         # Handle unexpected large sigmas
#         if sigma_range < 0 or sigma_range > 255:
#             rospy.logerr("[%s] Sigma range value out of range!" % rospy.Time.now())
#             range_factor.sigma_range = 255  # Default value if out of range
#         # Handle unexpected low ranges by setting the range to zero with max sigma
#         if measured_range < 0:
#             rospy.logerr("[%s] Measured range value low out of range!" % rospy.Time.now())
#             range_factor.index_or_measured_range = 0  # Default value if out of range
#             range_factor.sigma_range = 255  # Default value if out of range
#         # Handle unexpected high ranges by setting the range to max with max sigma
#         elif measured_range > 65535:
#             rospy.logerr("[%s] Measured range value high out of range!" % rospy.Time.now())
#             range_factor.index_or_measured_range = 65535  # Default value if out of range
#             range_factor.sigma_range = 255  # Default value if out of range
#         # Otherwise, set the measured range and sigma range
#         else:
#             range_factor.index_or_measured_range = measured_range
#             range_factor.sigma_range = sigma_range
#     else:
#         rospy.logerr("[%s] No index or measured range provided!" % rospy.Time.now())
#         return RangeFactor()
#     depth = int(depth * CODEC_SCALE_FACTORS["range_factor"]["depth"])
#     # Verify that the depth is within range
#     if depth < 0 or depth > 255:
#         rospy.logerr("[%s] Depth value out of range!" % rospy.Time.now())
#         range_factor.depth = 0 # note this is much more likely to be a negative, when surfacing than 12.7m
#     else:
#         range_factor.depth = depth
#     return range_factor

# Graph Update decoding:
def decode_pwc_from_int(encoded_pose):
    """This function decodes the BetweenFactor data from a message into a PoseWithCovariance
    Args:
        encoded_pose (list): The encoded pose data [x,y,z, qw, qx, qy, qz, sigma_x, sigma_y, sigma_z, sigma_psi, rho_xy, rho_xpsi, rho_ypsi]"""
    # Initialize the output message
    pwc = PoseWithCovariance()
    # Decode the position
    pwc.pose.position.x = encoded_pose[0] / CODEC_SCALE_FACTORS["between_factor"]["x"]
    pwc.pose.position.y = encoded_pose[1] / CODEC_SCALE_FACTORS["between_factor"]["y"]
    pwc.pose.position.z = encoded_pose[2] / CODEC_SCALE_FACTORS["between_factor"]["z"]
    # Build the orientation quaternion
    pwc.pose.orientation.x = encoded_pose[4] / CODEC_SCALE_FACTORS["between_factor"]["qw"]
    pwc.pose.orientation.y = encoded_pose[5] / CODEC_SCALE_FACTORS["between_factor"]["qx"]
    pwc.pose.orientation.z = encoded_pose[6] / CODEC_SCALE_FACTORS["between_factor"]["qy"]
    pwc.pose.orientation.w = encoded_pose[3] / CODEC_SCALE_FACTORS["between_factor"]["qz"]
    # Build the new covariance matrix
    sigma_x = encoded_pose[7] / CODEC_SCALE_FACTORS["between_factor"]["sigma_x"]
    sigma_y = encoded_pose[8] / CODEC_SCALE_FACTORS["between_factor"]["sigma_y"]
    sigma_z = encoded_pose[9] / CODEC_SCALE_FACTORS["between_factor"]["sigma_z"]
    sigma_psi = encoded_pose[10] / CODEC_SCALE_FACTORS["between_factor"]["sigma_psi"]
    rho_xy = encoded_pose[11] / CODEC_SCALE_FACTORS["between_factor"]["rho_xy"] if sigma_x and sigma_y else 0.0
    rho_xpsi = encoded_pose[12] / CODEC_SCALE_FACTORS["between_factor"]["rho_xpsi"] if sigma_x and sigma_psi else 0.0
    rho_ypsi = encoded_pose[13] / CODEC_SCALE_FACTORS["between_factor"]["rho_ypsi"] if sigma_y and sigma_psi else 0.0

    # Build the covariance matrix
    pwc.covariance = np.zeros(36)
    pwc.covariance[0] = sigma_x ** 2  # x
    pwc.covariance[7] = sigma_y ** 2  # y
    pwc.covariance[14] = sigma_z ** 2  # z
    pwc.covariance[21] = sigma_x ** 2  # roll
    pwc.covariance[28] = sigma_y ** 2  # pitch
    pwc.covariance[35] = sigma_psi ** 2  # yaw
    # Fill in the correlation coefficients
    pwc.covariance[1] = rho_xy * sigma_x * sigma_y  # sigma_xy
    pwc.covariance[5] = rho_xpsi * sigma_x * sigma_psi  # sigma_xpsi
    pwc.covariance[11] = rho_ypsi * sigma_y * sigma_psi  # sigma_ypsi
    # Fill the symmetric entries for var_xy, var_xpsi, var_ypsi
    pwc.covariance[6] = pwc.covariance[1]  # sigma_xy
    pwc.covariance[30] = pwc.covariance[5]  # sigma_xpsi
    pwc.covariance[31] = pwc.covariance[11]  # sigma_ypsi
    # Return the PoseWithCovariance message
    return pwc

def decode_range_event_from_int(encoded_range):
    """ Decode the RangeFactor data from the message, outputting the index, measured_range, range_sigma, and depth
    Args:
        encoded_range (list): The encoded range data [remote_address, index_or_measured_range, sigma_range, depth]"""
    # Decode the remote address
    remote_address = encoded_range[0]
    # Decode the index or measured range
    if encoded_range[2] == 0:  # If sigma_range is zero, it means an index is provided but not a measured range
        index = encoded_range[1] / CODEC_SCALE_FACTORS["range_factor"]["index"]
        measured_range = None
        sigma_range = None
    else:  # Otherwise, it means measured range is provided
        measured_range = float(encoded_range[1] / CODEC_SCALE_FACTORS["range_factor"]["measured_range"])
        sigma_range = float(encoded_range[2] / CODEC_SCALE_FACTORS["range_factor"]["sigma_range"])
        index = None  # No index provided, only measured range
    # Decode the depths
    depth = float(encoded_range[3] / CODEC_SCALE_FACTORS["range_factor"]["depth"])
    return remote_address, index, measured_range, sigma_range, depth

# Legacy Pre-encoding and decoding functions:
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

def encode_partial_graph_pose_as_int(position, orientation, sigmas):
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

def encode_partial_graph_range_as_int(range_value):
    """This function encodes the range value into a message
    Args:
        range_value (float): The range value
    Returns:
        int: The encoded range value
    """
    # Range is a int8, multiply by scale, then convert to int8
    range_value = int(range_value * CODEC_SCALE_FACTORS["partial_graph"]["range"])
    # verify all fields are within int8 range
    if not (np.abs(range_value) <= 127):
        rospy.logerr("[%s] Range value out of range!" % rospy.Time.now())
        return None
    else:
        return range_value

def decode_partial_graph_pose_from_int(position, orientation, sigmas):
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

def decode_partial_graph_range_from_int(range_value):
    """This function decodes the range value from a message
    Args:
        range_value (int): The range value
    Returns:
        float: The decoded range value
    """
    # Decode the data by applying the reverse of the encoding process
    # Range is a int8, divide by scale to get original value
    range_value = range_value / CODEC_SCALE_FACTORS["partial_graph"]["range"]
    return range_value

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