# Note this is 115bytes (by field encoding)
# I'd like to get it to one packet, but I think I'd have to encode each field individually with a min-max range

# Key Chain (4 bytes)
uint8 local_addr    # the int representation of the agent symbol
uint16 full_index   # the int pose index (up to 65k)
uint8 num_poses     # the int number of poses in this msg (0-6) (assumed consecutive)

# Pose Chain (6)
# Relative Pose 0 (16 bytes)
int16[3] relative_pos_0
int8[4] relative_rot_0
uint8[6] unique_sigmas_0
# Relative Pose 1 (16 bytes)
int16[3] relative_pos_1
int8[4] relative_rot_1
uint8[3] unique_sigmas_1
# Relative Pose 2 (16 bytes)
int16[3] relative_pos_2
int8[4] relative_rot_2
uint8[3] unique_sigmas_2
# Relative Pose 3 (16 bytes)
int16[3] relative_pos_3
int8[4] relative_rot_3
uint8[3] unique_sigmas_3
# Relative Pose 4 (16 bytes)
int16[3] relative_pos_4
int8[4] relative_rot_4
uint8[3] unique_sigmas_4
# Relative Pose 5 (16 bytes)
int16[3] relative_pos_5
int8[4] relative_rot_5
uint8[3] unique_sigmas_5

# Initiated Ranges (16 bytes)
# Initiated Range 0
uint8 local_index_0
uint8 remote_addr_0
uint16 meas_range_0
# Initiated Range 1
uint8 local_index_1
uint8 remote_addr_1
uint16 meas_range_1
# Initiated Range 2
uint8 local_index_2
uint8 remote_addr_2
uint16 meas_range_2
# Initiated Range 3
uint8 local_index_3
uint8 remote_addr_3
uint16 meas_range_3

# Recieved Ranges (8 bytes)
# Recieved Range 0
uint8 local_index_4
uint8 remote_addr_4
uint16 remote_index_4
# Recieved Range 1
uint8 local_index_5
uint8 remote_addr_5
uint16 remote_index_5