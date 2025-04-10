#! /usr/bin/env python3

'''
tests for using ping payload for ros msg transport
'''
import hashlib
import os
import secrets
import sys
import time
import traceback
import threading
import logging

import pytest
import rospy
import rospkg
import roslaunch

from std_msgs.msg import Header, Bool, UInt8
from ros_acomms_msgs.srv import PingModem, PingModemRequest, PingModemResponse
from ros_acomms_msgs.msg import PingReply

class TestUsePingPayload:
    '''tdma and modem_driver ping and auto ping test class

    test cases:
    '''
    VALID_TDMA_TYPES = ['tdma_advanced', 'tdma_scripted', 'tdma_slotted_aloha']
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """
        self.test_counter = 0
        rospy.init_node("test_use_ping_payload", log_level=rospy.DEBUG)
        self.logger = logging.getLogger("rosout")

        self.tdma_type = rospy.get_param("/tdma_type", "tdma_advanced")
        self.sim = rospy.get_param("/sim", True)
        if self.tdma_type not in TestUsePingPayload.VALID_TDMA_TYPES:
            raise NotImplementedError(f'This test module does not work with {self.tdma_type}. Valid tdma_types: {TestUsePingPayload.VALID_TDMA_TYPES}')

        self.roslaunch_dict = dict(
                sim=self.sim,
                tdma_type=self.tdma_type,
            )
        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/test_use_ping_payload.launch",
        )

        roslaunch_args = [f"{k}:={v}" for k, v in self.roslaunch_dict.items()]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()

        rospy.sleep(5.0)
        self.logger.warning('Node: test_use_ping_payload running!')

    def test_modem0_disabled(self):
        self.logger.warning(f'Testing use ping payload on modem0 which has ~use_ping_payload set to false')

        publisher = rospy.Publisher('/modem0/test_msg', UInt8, queue_size=5, latch=True)
        time.sleep(2)
        msg = UInt8(data=1)
        publisher.publish(msg)

        try:
            ret = rospy.wait_for_message(f'/modem1/from_acomms/test_msg', UInt8, timeout=30)
        except rospy.exceptions.ROSException:
            self.logger.warning("Success! timed out while waiting for /modem1/from_acomms/test_msg")
        else:
            self.logger.warning("Got test_msg from modem0 which has ~use_ping_payload:False and no comms_slots.")
            pytest.fail("Got test_msg from modem0 which has ~use_ping_payload:False and no comms_slots.")

    def test_modem1_enabled(self):
        self.logger.warning(f'Testing use ping payload on modem1 which has ~use_ping_payload set to true')

        publisher = rospy.Publisher('/modem1/test_msg', UInt8, queue_size=5, latch=True)
        time.sleep(2)
        msg = UInt8(data=1)
        publisher.publish(msg)

        try:
            ret = rospy.wait_for_message(f'/modem0/from_acomms/test_msg', UInt8, timeout=30)
        except rospy.exceptions.ROSException:
            self.logger.warning("Timed-out waiting for msg on /modem0/from_acomms/test_msg from modem1")
            pytest.fail("Timed-out waiting for msg on /modem0/from_acomms/test_msg from modem1")
        else:
            if ret.data == 1:
                self.logger.warning("Success! Got msg on /modem0/from_acomms/test_msg from modem1 which has ~use_ping_payload:True and no comms_slots")

    def test_modem0_ping_modem(self):
        self.logger.warning(f'Testing ping_modem service endpoint in modem_driver')

        rospy.wait_for_service('/modem0/ping_modem', timeout=100)
        send_modem_ping = rospy.ServiceProxy('/modem0/ping_modem', PingModem)

        try:
            req = PingModemRequest(dest=2, rate=1, timeout_sec=1000.0)
            resp = send_modem_ping(req)
        except rospy.exceptions.ROSException:
            err = "timed-out during request?"
            self.logger.warning(err)
            pytest.fail(err)
        else:
            if resp.timed_out:
                err = "response timed out!"
                self.logger.warning(err)
                pytest.fail(err)

            self.logger.warning(f"Success! we got a ping_reply via the modem_driver service endpoint: resp: {resp}")

    def test_modem0_tdma_ping_modem(self):
        self.logger.warning(f'Testing ping_modem service endpoint in tdma')

        rospy.wait_for_service('/modem0/tdma/ping_modem', timeout=100)
        send_modem_ping = rospy.ServiceProxy('/modem0/tdma/ping_modem', PingModem)

        try:
            req = PingModemRequest(dest=2, rate=1, timeout_sec=1000.0)
            resp = send_modem_ping(req)
        except rospy.exceptions.ROSException:
            err = "timed-out during request?"
            self.logger.warning(err)
            pytest.fail(err)
        else:
            if resp.timed_out:
                err = "response timed out!"
                self.logger.warning(err)
                pytest.fail(err)

            self.logger.warning(f"Success! we got a ping_reply via the tdma service endpoint: resp: {resp}")

    def test_modem1_change_auto_ping_src(self):
        self.logger.warning(f'Testing change_auto_ping_src')

        publisher = rospy.Publisher('/modem1/tdma/change_auto_ping_modem_src', UInt8, queue_size=5, latch=True)
        msg = UInt8(data=2)
        publisher.publish(msg)
        time.sleep(2.0)
        self.logger.warning(f'Published to /modem1/tdma/change_auto_ping_modem_src, msg: {msg}')
        ping_modem_src = rospy.get_param('/modem1/tdma/ping_modem_src')
        self.logger.warning(f'Query param server, ping_modem_src: {ping_modem_src}')
        if ping_modem_src != 2:
            err = f"Failed setting auto ping_modem_src to 2!"
            self.logger.warning(err)
            pytest.fail(err)

        try:
            ret = rospy.wait_for_message(f'/modem1/ping_reply', PingReply, timeout=1000)
            # incase we got the last ping_reply from our prior auto ping src
            ret = rospy.wait_for_message(f'/modem1/ping_reply', PingReply, timeout=1000)
        except rospy.exceptions.ROSException:
            err = f"Failed! Timed-out waiting for ping_reply on modem1 from modem2"
            self.logger.warning(err)
            pytest.fail(err)
        else:
            if ret.src == 2:
                self.logger.warning("Success! got ping_reply for modem1 from modem2")
            else:
                err = f"Failed! Got ping_reply from ret.src: {ret.src} not modem2"
                self.logger.warning(err)
                pytest.fail(err)

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """
        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
