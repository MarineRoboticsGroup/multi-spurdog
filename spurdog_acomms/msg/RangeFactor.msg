# RangeFactor.msg
# Used for building the graph update over acomms
uint8 remote_address
uint16 index_or_measured_range # This is indicated by the presence of a sigma_range
uint8 sigma_range
uint8 depth