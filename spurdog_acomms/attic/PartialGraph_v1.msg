# Key Chain
uint8 local_addr    # the int representation of the agent symbol
uint16 full_index   # the int pose index (up to 65k)
uint8 num_poses     # the int number of poses in this msg (0-6) (assumed consecutive)
# Relative Pose (max 6)
RelativePoseMin rel_pose_0
RelativePoseMin rel_pose_1
RelativePoseMin rel_pose_2
RelativePoseMin rel_pose_3
RelativePoseMin rel_pose_4
RelativePoseMin rel_pose_5
# Initiated Ranges (max 4)
InitiatedRange init_range_0
InitiatedRange init_range_1
InitiatedRange init_range_2
InitiatedRange init_range_3
# Recieved Ranges (max 2)
RecievedRange rcvd_range_0
RecievedRange rcvd_range_1
# Common Sigma (max 1)
uint8 sigma_range