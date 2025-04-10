# tdma base class for tdma_advanced
# we_are_active, current_slot, remaining_slot_seconds, remaining_active_seconds, time_to_next_active
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
