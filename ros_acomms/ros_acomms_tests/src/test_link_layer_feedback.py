#! /usr/bin/env python3

''' Test the Link Layer Feedback node'''

import os
import sys
import pytest
import rospy
import rospkg
import roslaunch
from std_msgs.msg import UInt8
import time

from ros_acomms_msgs.srv import GetNextQueuedMessage, GetNextQueuedMessageRequest, GetNextQueuedMessageResponse
from ros_acomms_msgs.msg import UpdateQueuedMessage, LinkStatsFeedback, SST

class TestLinkLayerFeedback:
    '''ros_acomms LinkLayerFeedback test class'''
    
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """

        rospy.init_node("test_link_layer_feedvack", log_level=rospy.INFO)

        tdma_type = rospy.get_param("/tdma_type", "tdma")

        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/link_layer_feedback_test.launch",
        )

        roslaunch_args = [f"tdma_type:={tdma_type}"]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start() 

    def test_feedback(self):
        '''Quick test to verify that we get a feedback message.'''

        test_message_publisher = rospy.Publisher('/modem0/test_msg', UInt8, queue_size=5)
        time.sleep(2)

        # Wait for an SST message to be published
        try:
            sst = rospy.wait_for_message("/modem1/sst", SST, timeout=21)
        except rospy.exceptions.ROSException:
            pytest.fail("timed out when waiting for SST message")

        msg = UInt8()

        test_message_publisher.publish(msg)

        try:
            inbound_test_message = rospy.wait_for_message("/modem1/from_acomms/test_msg", UInt8, timeout=60)
        except rospy.exceptions.ROSException:
            pytest.fail("timed out when waiting for test message")

        # Now wait on the feedback message
        try:
            feedback_message = rospy.wait_for_message("/modem0/link_layer_feedback", LinkStatsFeedback, timeout=60)
        except rospy.exceptions.ROSException:
            pytest.fail("timed out when waiting for feedback message")

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """

        self.launch.shutdown()


if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
