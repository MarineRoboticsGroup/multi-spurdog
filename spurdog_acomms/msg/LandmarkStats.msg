# Per-landmark statistics message
Header header

# Landmark identification
string landmark_name  # e.g., "L0", "L1"
int16 landmark_src    # e.g., 1, 2 (or 8, 9 with your new code)

# Ping statistics (since node start or last reset)
uint32 pings_sent_to_landmark
uint32 ping_replies_received
float32 success_rate_percent

# Recent performance (last N pings, e.g., last 10)
uint32 recent_window_size
uint32 recent_replies_received
float32 recent_success_rate_percent

# Signal quality (from most recent successful ping)
float32 last_snr_in
float32 last_snr_out
float32 last_owtt
float32 last_range_meters
time last_successful_ping_time

# Health indicators
duration time_since_last_reply
bool is_responsive  # true if replied within last X seconds (e.g., 30s)
