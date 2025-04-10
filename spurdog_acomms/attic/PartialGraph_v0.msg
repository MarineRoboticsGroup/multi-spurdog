Symbol first_key 	#first key in the local pose chain
uint8 last_key 	#the integer difference between the first and last key index
uint8 src_range_key	#the key index (0<n<last) when the range occurs
Symbol blind_range_key	#the key at the other agent
PoseChain pose_chain	# 4 relative poses with noise models
uint16 range_src_dest	
uint16 range_src_blind
uint8 sigma_range
