Header header
int32 step_id
uint16 status # TODO: Add Overflow (bit 3) Out of sequence (bit 2)
duration duration

# Status
uint16 STATUS_RESET = 1
uint16 STATUS_SEQUENCING = 2
uint16 STATUS_OVERFLOW = 4