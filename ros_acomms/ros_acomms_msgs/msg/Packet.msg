int16 src
int16 dest

int8 PACKET_TYPE_UNKNOWN = -1
int8 PACKET_TYPE_AUTO = 0
int8 PACKET_TYPE_FSK = 1
int8 PACKET_TYPE_LEGACY_PSK = 2
int8 PACKET_TYPE_FDP = 3
int8 packet_type

int8 miniframe_rate
int8 dataframe_rate
uint8[] miniframe_bytes
uint8[] dataframe_bytes