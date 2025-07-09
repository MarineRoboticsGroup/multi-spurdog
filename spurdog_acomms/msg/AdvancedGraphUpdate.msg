# GraphUpdate.msg
# Header Info (4 bytes)
uint8 sender_address
uint16 first_key_index
uint8 num_poses
# Recovery Prior 0 (12 bytes)
int16 pose_prior_0_x
int16 pose_prior_0_y
int8 pose_prior_0_z
int8 pose_prior_0_qw
int8 pose_prior_0_qx
int8 pose_prior_0_qy
int8 pose_prior_0_qz
uint8 pose_prior_0_sigma_x
uint8 pose_prior_0_sigma_y
uint8 pose_prior_0_sigma_z
uint8 pose_prior_0_sigma_psi
# Between Factor 0 (13 bytes)
int16 relative_pose_0_x
int16 relative_pose_0_y
int8 relative_pose_0_z
int8 relative_pose_0_qw
int8 relative_pose_0_qx
int8 relative_pose_0_qy
int8 relative_pose_0_qz
uint8 relative_pose_0_sigma_x
uint8 relative_pose_0_sigma_y
uint8 relative_pose_0_sigma_z
uint8 relative_pose_0_sigma_psi
# Between Factor 1 (13 bytes)
int16 relative_pose_1_x
int16 relative_pose_1_y
int8 relative_pose_1_z
int8 relative_pose_1_qw
int8 relative_pose_1_qx
int8 relative_pose_1_qy
int8 relative_pose_1_qz
uint8 relative_pose_1_sigma_x
uint8 relative_pose_1_sigma_y
uint8 relative_pose_1_sigma_z
uint8 relative_pose_1_sigma_psi
# Between Factor 2 (13 bytes)
int16 relative_pose_2_x
int16 relative_pose_2_y
int8 relative_pose_2_z
int8 relative_pose_2_qw
int8 relative_pose_2_qx
int8 relative_pose_2_qy
int8 relative_pose_2_qz
uint8 relative_pose_2_sigma_x
uint8 relative_pose_2_sigma_y
uint8 relative_pose_2_sigma_z
uint8 relative_pose_2_sigma_psi
# Between Factor 3 (13 bytes)
int16 relative_pose_3_x
int16 relative_pose_3_y
int8 relative_pose_3_z
int8 relative_pose_3_qw
int8 relative_pose_3_qx
int8 relative_pose_3_qy
int8 relative_pose_3_qz
uint8 relative_pose_3_sigma_x
uint8 relative_pose_3_sigma_y
uint8 relative_pose_3_sigma_z
uint8 relative_pose_3_sigma_psi
# Range Factor 0 (5 bytes)
uint8 range_event_0_remote_address
uint16 range_event_0_index_or_measured_range
uint8 range_event_0_sigma_range
uint8 range_event_0_depth
# Range Factor 1 (5 bytes)
uint8 range_event_1_remote_address
uint16 range_event_1_index_or_measured_range
uint8 range_event_1_sigma_range
uint8 range_event_1_depth
# Range Factor 2 (5 bytes)
uint8 range_event_2_remote_address
uint16 range_event_2_index_or_measured_range
uint8 range_event_2_sigma_range
uint8 range_event_2_depth
# Range Factor 3 (5 bytes)
uint8 range_event_3_remote_address
uint16 range_event_3_index_or_measured_range
uint8 range_event_3_sigma_range
uint8 range_event_3_depth