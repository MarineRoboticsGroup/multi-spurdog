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
# from ros_acomms_msgs.msg import ReceivedPacket
from ros_acomms_msgs.srv import PingTranspondersRequest, PingTranspondersResponse
from ros_acomms_tests.msg import FragmentationTestMsg


class TestRosAcomms:
    '''ros_acomms test class'''

    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """
        self.test_counter = 0
        rospy.init_node("ros_acomms_tests", log_level=rospy.INFO)

        tdma_type = rospy.get_param("/tdma_type", "tdma")

        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/test.launch",
        )

        roslaunch_args = [f"tdma_type:={tdma_type}"]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()
        time.sleep(5)

    def test_queue(self):
        '''test_queue sends 3 messages through the ros_acomms and verifies that all 3 of the messages are recieved'''
        print("Starting Test: test_ros_acomms.py / test_queue")

        queue_publisher = rospy.Publisher('/modem0/test_msg', UInt8, queue_size=5)
        time.sleep(2)

        return_arr = []
        input_arr = []

        msg = UInt8()

        try:
            for val in range(3):
                msg.data = val
                queue_publisher.publish(msg)
                ret = rospy.wait_for_message(
                    "/modem1/from_acomms/test_msg", UInt8, timeout=100)

                return_arr.append(ret.data)
                input_arr.append(msg.data)

            assert return_arr == input_arr, f"return_arr: {return_arr} != input_arr: {input_arr}"

        except rospy.exceptions.ROSException:
            pytest.fail("timed out when waiting for message")

    def test_rossrv_as_rosmsg(self):
        """
        test_rossrv_as_rosmsg tests pub/sub using the ROSSRV Message Class
        1. modem0 sends ROSSRV Request as ROSMSG to modem1, verifies receipt of ROSSRV Request type
        2. modem1 responds to modem0 with ROSSRV Response as ROSMSG, verifies receipt of ROSSRV Response type
        """

        print("Starting Test: test_ros_acomms.py / test_rossrv_as_rosmsg")

        try:
            rossrv_cmd = rospy.Publisher('/modem0/command',
                                         PingTranspondersRequest, queue_size=5)
            rossrv_resp = rospy.Publisher('/modem1/response',
                                          PingTranspondersResponse, queue_size=5)
            time.sleep(1)

            # Prep CMD Request from modem0 --> modem1
            cmd_request = PingTranspondersRequest(transponder_dest_mask=[True, True, True, True],
                                                  timeout_sec=0.0)
            rossrv_cmd.publish(cmd_request)

            # Wait for CMD @ modem1 via ACOMMs...
            recv_cmd = rospy.wait_for_message('/modem1/from_acomms/command',
                                              PingTranspondersRequest,
                                              timeout=100)
            # Verify RX'd Class Type
            assert isinstance(recv_cmd, PingTranspondersRequest), (f"Received Cmd [{type(recv_cmd).__name__}] "
                                                                   f"(looking for <PingTranspondersRequest>)")

            # Prep CMD Response from modem1 --> modem0
            response = PingTranspondersResponse(travel_times=[0.0, 0.0, 0.0, 0.0])
            rossrv_resp.publish(response)

            # Wait for RESPONSE @ modem0 via ACOMMs...
            recv_resp = rospy.wait_for_message('/modem0/from_acomms/response',
                                               PingTranspondersResponse,
                                               timeout=100)
            # Verify RX'd Class Type
            assert isinstance(recv_resp, PingTranspondersResponse), (f"Received Cmd [{type(recv_resp).__name__}] "
                                                                     f"(looking for <PingTranspondersResponse>)")
        except rospy.exceptions.ROSException as exc:
            pytest.fail(f"failed registering ROSSRV msgs for pub/sub {(exc,)}")

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """

        self.launch.shutdown()


if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
