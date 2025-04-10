# Standard ROS header containing timestamp and frame information
Header header

# The source ID of the message
int16 src

# The destination ID of the message
int16 dest

# The time when the message was sent from the source
time src_time

# The time when the message was received by the destination
time dest_time

# The offset in microseconds between the source and destination times
uint32 src_dest_offset_us
