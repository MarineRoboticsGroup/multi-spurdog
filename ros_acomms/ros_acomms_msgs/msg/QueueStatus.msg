Header header

# Number of messages in all queues
int32 total_message_count
# Highest priority of any message, in any queue
int16 highest_priority
# Set of all destination addresses to which queued messages are addressed
int16[] queued_dest_addresses

# Summary of outstanding messages, grouped by (priority, destination)
SummaryMessageCount[] summary_message_counts

# Summary of outstanding messages, per-queue.
QueueSummary[] queue_summaries
