# Acoustic transmit parameters
##############################

# Source Latitude [decimal degrees]. Positive is north of equator; negative is south
# (-90 <= latitude <= +90).
float32 src_latitude

# Source Longitude [decimal degrees]. Positive is east of prime meridian; negative is west
# (-180 <= longitude <= +180)
float32 src_longitude

# Source Depth [m]. Positive is below the WGS 84 ellipsoid.
float32 src_depth

# Source transmit level [dB]
float32 src_tx_level_db

# Center frequency [hz]
float32 center_frequency_hz

# Bandwidth [hz]
float32 bandwidth_hz

# Transmit time [time]. Unix epoch time.
# Seconds since epoch, nanoseconds since second. See rospy.rostime
time transmit_time

# Transmit duration [duration].
# Seconds and nanoseconds. See rospy.duration
duration transmit_duration

#########################
# Packet                #
# ros_acomms Packet.msg #
#########################
ros_acomms_msgs/Packet packet

############################
# PingReply                #
# ros_acomms PingReply.msg #
############################
int8 PING_TYPE_NONE = 0
int8 PING_TYPE_REQUEST = 1
int8 PING_TYPE_REPLY = 2

bool is_transponder
int8 ping_type
time ping_transaction_start
ros_acomms_msgs/PingReply ping_reply