#!/usr/bin/env python3

from typing import Dict, List, Tuple, Optional

import rospy
import rospkg
from std_msgs.msg import Header
from ros_acomms_msgs.msg import CST, SST, LinkStatsFeedback

from ros_acomms_msgs.srv import GetNextQueuedMessage, GetNextQueuedMessageRequest, GetNextQueuedMessageResponse
from ros_acomms_msgs.msg import UpdateQueuedMessage

from ltcodecs import RosMessageCodec

class LinkLayerFeedbackNode:
    def __init__(self):
        rospy.init_node('link_layer_feedback')

        self.last_sst = None
        self.most_recent_csts = {}
        self.ssts_for_csts = {}
        self.last_update_time = rospy.get_time()

        self.minimum_update_interval_seconds = rospy.get_param('~minimum_update_interval_seconds', 60)
        self.destination_address = rospy.get_param('~destination_address', 121)
        self.message_priority = rospy.get_param('~message_priority', 10)

        rospack = rospkg.RosPack()
        link_stats_feedback_codec_file = rospy.get_param('~link_stats_feedback_codec_file',
                                                rospack.get_path('ros_acomms_msgs') + '/codecs/LinkStatsFeedback.yaml')
        rospy.loginfo(f"link_stats_feedback_codec file path: {link_stats_feedback_codec_file}")
        self.link_stats_feedback_codec: RosMessageCodec = RosMessageCodec.from_codec_file(LinkStatsFeedback, link_stats_feedback_codec_file)

        rospy.Service('get_next_link_feedback_message', GetNextQueuedMessage, self.handle_get_next)
        rospy.Subscriber('update_link_feedback_queue', UpdateQueuedMessage, self.on_update_queue)

        rospy.Subscriber("cst", CST, self.on_cst)
        rospy.Subscriber("sst", SST, self.on_sst)

        rospy.loginfo("link_stats_feedback running")

        rospy.spin()

    def on_cst(self, cst_msg: CST) -> None:
        if not self.last_sst:
            rospy.logwarn_throttle(10,
                                   "No SST information is available, not saving CST")
            return

        # We track the most recent CSTs associated with each sender
        # Only update if the header decoded (so we have a src address) and it's a PSK packet (3, 4, or 5)
        if cst_msg.src >= 0 and cst_msg.packet_type in (3, 4, 5):
            self.ssts_for_csts[cst_msg.src] = self.last_sst
            # Hijack the noise value in the CST message and insert the SST median
            self.most_recent_csts[cst_msg.src] = cst_msg

    def on_sst(self, sst_msg: SST) -> None:
        self.last_sst = sst_msg

    def handle_get_next(self, request: GetNextQueuedMessageRequest) -> GetNextQueuedMessageResponse:
        # First, check to see if we are locked out because we just sent one
        if rospy.get_time() - self.last_update_time < self.minimum_update_interval_seconds:
            return GetNextQueuedMessageResponse(has_message=False)

        # We only ever have one message to queue.  If one is already queued, don't queue another.
        if 1 in request.exclude_message_ids:
            return GetNextQueuedMessageResponse(has_message=False)

        if not self.last_sst:
            rospy.logwarn_throttle(10, "Unable to generate LinkStatsFeedback message because no SST information is available")
            return GetNextQueuedMessageResponse(has_message=False)

        # Sort the lists so that the noise value order matches the cst order
        sorted_csts = [v for k, v in sorted(self.most_recent_csts.items())]
        sorted_noise = [v.summary_median for k, v in sorted(self.ssts_for_csts.items())]

        # Create the new message
        new_feedback_msg = LinkStatsFeedback(header=Header(stamp=rospy.Time.now()),
                                             median_noise_dB=self.last_sst.summary_median,
                                             csts=sorted_csts,
                                             noise_for_csts=sorted_noise)

        # encode it
        message_bits, message_metadata = self.link_stats_feedback_codec.encode(new_feedback_msg)

        # Make sure it is small enough
        if len(message_bits) > request.max_size_in_bits:
            return GetNextQueuedMessageResponse(has_message=False)

        return GetNextQueuedMessageResponse(has_message=True,
                                            message_id=1,
                                            dest_address=self.destination_address,
                                            priority=self.message_priority,
                                            data=message_bits.tobytes(),
                                            data_size_in_bits=len(message_bits)
                                            )

    def on_update_queue(self, update_msg: UpdateQueuedMessage) -> None:
        # We assume that we're only dealing with one of these messages at a time, and that we don't get any
        # CSTs while we're dealing with an outbound transmission.
        # So, all we need to handle here is the transmit update, and we flush the CST cache when we do
        if update_msg.event == UpdateQueuedMessage.EVENT_LINK_TRANSMITTED:
            self.most_recent_csts.clear()
            self.ssts_for_csts.clear()
            rospy.loginfo("Link feedback message was transmitted, cleared CST cache")


if __name__ == "__main__":
    import traceback
    try:
        node = LinkLayerFeedbackNode()
        rospy.loginfo("LinkLayerFeedbackNode shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("LinkLayerFeedbackNode shutdown (interrupt)")
    except:
        print(traceback.format_exc())
        rospy.loginfo(f"LinkLayerFeedbackNode shutdown. Thrown unhandled exception: {traceback.format_exc()}")
