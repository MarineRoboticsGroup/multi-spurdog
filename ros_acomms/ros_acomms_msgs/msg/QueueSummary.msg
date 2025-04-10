
# Queue that the topic corresponds to
int16 queue_id
# Name of subscribed topic
string topic
# Current priority of each queue
# However, it is possible for an individual queued message to be added
# with a different priority.
int16 priority
# Whether the queue is active or not
bool enabled
# How many un-sent messages are in the queue
int16 message_count
# The size in bits of the next message in this queue
int32 next_message_size_bits
# The total size of all messages in this queue (in bits)
int32 total_bits_in_queue