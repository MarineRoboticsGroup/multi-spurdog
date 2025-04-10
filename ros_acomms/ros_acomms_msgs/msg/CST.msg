# CST message Version number; if it's less than 6, then it's version 0
int8 version_number

# Indication of the Received Packet's State: 0 if good, 1 if bad CRCs, 2 if packet timeout
int8 mode

# Time of Arrival of the Packet
time toa

# TOA mode Clock status represented as bitwise field:
# Bits 0:1: RTC source - 0: None, 1: Onboard RTC, 2: User Command (CLK), 3: User GPS (GPRMC)
# Bits 2:4 PPS Source - 0: None, 1: RTC, 2: CAL (future), 3: EXT (from external pin), 4: EXT_SYNC (from RTC synchronized to the last EXT PPS pulse)
int8 toa_mode

# Matched-filter peak value
uint16 mfd_peak

# MFPK compensated for shift and converted to dB
int16 mfd_pow

# Ratio of MFPK and incoherent broad-band noise used for the detector test
int16 mfd_ratio

# Sound Pressure Level at the receiver in dB re. 1 µPa
int16 mfd_spl

# Analog gain setting
int16 agn

# Shift of previous input buffer
int16 shift_ainp

# Shift of current input buffer
int16 shift_ain

# Shift of the basebanded buffer containing a detection
int16 shift_mfd

# Shift performed during basebanding incoming data
int16 shift_p2b

# Packet Rate (or miniframe rate for FDP)
int8 rate_num

# Packet Source ID
int16 src

# Packet Destination ID
int16 dest

# PSK Error Code:
# 0: No Error
# 1: Bad Modulation header
# 2: Bad CRC, Data Header
# 3: Bad CRC on any frame
# 4: Error accessing coproc - if modem loses r/w connection to the coproc
# 5: Equalizer timeout - if the coproc never returns
# 6: Missed start of PSK packet
int16 psk_error

# Packet Type:
# -1: Unknown
# 1: FSK
# 2: FSK Mini
# 3: PSK
# 4: PSK Mini
# 5: PSK FDP
int8 packet_type

# Number of Frames Expected
int16 num_frames

# Number of Frames with Bad CRCs
int16 bad_frames_num

# Received signal strength in dB re. 1 µPa
int16 snr_rss

# Input SNR, dB, channel 1 only, reported to 1 decimal place in version 6 of CACST
float32 snr_in

# SNR at the output of the equalizer, dB, reported to 1 decimal place in version 6 of CACST
float32 snr_out

# Symbol SNR for spread spectrum packets only, dB
float32 snr_sym

# Mean Squared Error from Equalizer
float32 mse

# Data Quality Factor, FSK only
int16 dqf

# Doppler shift measurement
float32 dop

# Std dev. of noise, dB
int16 noise

# Carrier frequency of received packet, Hz
int32 carrier

# Bandwidth of received packet, Hz
int32 bandwidth

# Sequence number of packet
int32 sequence_number

# Dataframe rate of packet
int16 data_rate

# Number of Data Frames in the packet
int16 num_data_frames

# Number of Bad Data Frames in the packet
int16 num_bad_data_frames