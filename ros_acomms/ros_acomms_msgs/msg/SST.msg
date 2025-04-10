# Timestamp, generated on computer (not modem)
time time

# SST message version
int8 sst_version

# In-water in-band Sound Pressure Level (SPL) in dB. This is the average SPL over the last num_samples
float32 in_water_spl_dB

# Detector used during SPL calculation.
# This enum represents the detector type (e.g., psk chan, fsk chan, etc...).  Only PSK is supported as of FW 3.2.2
int8 detector

# Number of baseband samples used for each measurement (length of MFD filter)
int16 num_samples

# sample minimum of SPL history (in dB).  Min of five-number summary (descriptive statistics)
float32 summary_min

# lower quartile of SPL history (in dB).  First quartile of five-number summary (descriptive statistics)
float32 summary_lower_quartile

# median value of SPL history (in dB).  Middle value of five-number summary (descriptive statistics)
float32 summary_median

# upper quartile of SPL history (in dB).  Third quartile of five-number summary (descriptive statistics)
float32 summary_upper_quartile

# sample maximum of SPL history (in dB).  Max of five-number summary (descriptive statistics)
float32 summary_max

# number of elements in SPL history.  Each element is the average of a unique set of num_samples of baseband data
int16 summary_len