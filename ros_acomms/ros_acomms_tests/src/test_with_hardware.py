#! /usr/bin/env python3

'''
tests for ros_acomms
'''
import hashlib
import os
import secrets
import sys
import time
import traceback

import pytest
import rospy
import rospkg
import roslaunch

from std_msgs.msg import UInt8


class TestRosAcommsSerial:
    '''ros_acomms test class'''

    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """
        self.test_counter = 0
        rospy.init_node("ros_acomms_test_serial", log_level=rospy.INFO)

        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/test_with_hardware.launch",
        )

        roslaunch_args = []
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()
        time.sleep(5)

    def test_queue(self):
        '''test_queue sends 3 messages through the ros_acomms and verifies that all 3 of the messages are recieved'''

        queue_publisher = rospy.Publisher('/modem0/test_msg', UInt8, queue_size=5)

        return_arr = []
        input_arr = []

        msg = UInt8()

        print("Waiting for modem drivers...")

        # Wait until the modem drivers are ready
        try:
            rospy.wait_for_service('/modem0/queue_tx_packet', timeout=30)
            rospy.wait_for_service('/modem1/queue_tx_packet', timeout=30)
        except rospy.exceptions.ROSException:
            pytest.fail("Timed out waiting for modem drivers to be ready")

        print("Modem drivers ready.")

        use_sim_time = rospy.get_param("/use_sim_time", None)
        if use_sim_time is not None:
            pytest.fail("Sim time is active, it should be off for the hardware test.")

        time.sleep(2)

        try:
            for val in range(3):
                msg.data = val
                queue_publisher.publish(msg)
                print(f"Published test message {val}")

                ret = rospy.wait_for_message(
                    "/modem1/from_acomms/test_msg", UInt8, timeout=60)

                return_arr.append(ret.data)
                input_arr.append(msg.data)

            assert return_arr == input_arr, f"return_arr: {return_arr} != input_arr: {input_arr}"

        except rospy.exceptions.ROSException:
            pytest.fail("timed out when waiting for packet")

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """

        self.launch.shutdown()


if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
