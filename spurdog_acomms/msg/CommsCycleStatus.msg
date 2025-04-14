uint16	sequence_number       	# Monotonically increasing cycle count
bool	init_complete		# True if the init process is complete
bool	cycle_complete        	# True if the cycle completed as expected (i.e. 3 of 3 Partial Graphs recieved)
bool	should_smooth		# True if we only recieved our message (as 1/2 or 1/3).
uint16	key			# The key index of the first pose in the scrubbed cycle
float32[3] translation		# The smoothed translation from a scrubbed cycle
float32[4] quaternion		# The smoothed quaternion orientation from a scrubbed cycle
float32[6] sigmas		# The covariance of smoothed relative pose
