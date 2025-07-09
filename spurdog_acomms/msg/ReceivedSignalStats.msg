Header header
# Acomms Header Info
time toa
int16 src
int16 dest
string signal_type
int32 carrier
int32 bandwidth
int8 rate_num
string packet_type
# Selected CST Data (always filled)
uint16 mfd_peak
int16 mfd_pow
int16 mfd_ratio
int16 mfd_spl
int16 snr_rss
float32 snr_in
float32 snr_out
float32 mse
float32 dop
int16 noise
# Selected Range Data (0 if not a range packet)
int8 tx_level
float32 owtt
float32 tat