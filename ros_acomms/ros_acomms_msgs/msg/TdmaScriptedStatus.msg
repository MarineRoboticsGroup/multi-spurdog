# tdma basic node info at root level for all sub classes (super class)
Header header

bool we_are_active
uint16 current_slot
int8 minimum_priority
string[] require_tags
string[] exclude_tags
float32 remaining_slot_seconds
float32 remaining_active_seconds
float32 time_to_next_active
float32 time_to_next_dedicated_slot
float32 slot_duration_seconds

# tdma advanced status (parent class)
bool software_mute
string message

int8 miniframe_rate
int8 dataframe_rate
int32 maximum_miniframe_bytes
int32 maximum_dataframe_bytes
uint16 num_slots
uint16[] active_slots
uint16[] nav_slots
uint16[] comms_slots

int8 pings_per_slot
int16 ping_modem_src
bool ping_transponders

# scripted status additional meta-data (child class)
int8 last_miniframe_rate
int8 last_dataframe_rate
int32 last_maximum_miniframe_bytes
int32 last_maximum_dataframe_bytes

int16 this_cycle_tx_count
int16 this_cycle_tx_total

float32 packet_duration_sec
string[] this_cycle_modem_commands
string[] last_cycle_modem_commands 
