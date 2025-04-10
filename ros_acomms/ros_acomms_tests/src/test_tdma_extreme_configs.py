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
import threading
import logging

import pytest
import rospy
import rospkg
import roslaunch

from std_msgs.msg import UInt8, String
from ros_acomms_msgs.msg import TdmaAdvancedStatus, TdmaScriptedStatus, TdmaSlottedAlohaStatus, ReceivedPacket, TransmittedPacket
from ros_acomms_tests.msg import FragmentationTestMsg

# from ros_acomms.mac_utils import expanded_slots_as_ranges

class TestTdmaExtremeConfigs:
    '''tdma Extended test class

    test cases:
'''
    VALID_TDMA_TYPES = ['tdma_advanced', 'tdma_scripted', 'tdma_slotted_aloha']
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """
        self.test_counter = 0
        rospy.init_node("test_tdma_extreme_configs", log_level=rospy.DEBUG)
        self.logger = logging.getLogger("rosout")

        self.tdma_type = rospy.get_param("/tdma_type", "tdma_slotted_aloha")
        if self.tdma_type not in TestTdmaExtremeConfigs.VALID_TDMA_TYPES:
            raise NotImplementedError(f'This test module does not work with {self.tdma_type}. Valid tdma_types: {TestTdmaExtremeConfigs.VALID_TDMA_TYPES}')
        if self.tdma_type == 'tdma_advanced':
            self.tdma_msg_type = ('tdma_advanced_status', TdmaAdvancedStatus)
        elif self.tdma_type == 'tdma_scripted':
            self.tdma_msg_type = ('tdma_scripted_status', TdmaScriptedStatus)
        elif self.tdma_type == 'tdma_slotted_aloha':
            self.tdma_msg_type = ('tdma_slotted_aloha_status', TdmaSlottedAlohaStatus)

        self.roslaunch_dict = dict(
                num_slots=(2 ** 16) - 1,
                slot_duration_seconds=300,
                secondary_modem_guard_time=294,
                active_slots='0:150:2',
                comms_slots='::2',
                nav_slots='::4',
                aloha_slots='100:152:2,280:10',
                tdma_type=self.tdma_type,
                test_cycling_rates=False,
            )

        self.modem0_last = None

        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/test_tdma_extreme_configs.launch",
        )

        roslaunch_args = [f"{k}:={v}" for k, v in self.roslaunch_dict.items()]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()

        self.one_cycle_duration = self.roslaunch_dict['slot_duration_seconds'] * self.roslaunch_dict['num_slots']
        self.logger.warning(f'test_tdma_extreme_configs one_cycle_duration ({self.one_cycle_duration} sec). Sleeping for 2.0')
        rospy.sleep(2.0)
        self.logger.warning('Node: test_tdma_extended running!')

    def _topic_callback(self, msg):
        topic = msg._connection_header['topic']
        if 'modem0' in topic:
            self.modem0_last = msg

    def test_extreme_slot_ranges(self):
        self.logger.warning(f'Testing Extreme Configs on modem0')
        self.logger.info(f'Testing Extreme Configs:: Sleep for 2.0 then start test on /modem0.')
        rospy.sleep(2.0)

        while not rospy.is_shutdown():
            try:
                self.logger.info(f'Testing Extreme Configs:: Waiting for next message on /modem0/{self.tdma_msg_type[0]}. timeout={self.roslaunch_dict["slot_duration_seconds"]}')
                ret = rospy.wait_for_message(
                    f'/modem0/{self.tdma_msg_type[0]}', self.tdma_msg_type[1], timeout=self.roslaunch_dict['slot_duration_seconds'])

                if self.tdma_type == 'tdma_slotted_aloha':
                    # assert str(expanded_slots_as_ranges(ret.nav_slots)) == '12-96:4', f'ERROR: nav_slots not expected range: 12-96:4, aloha_slots should trim nav_slots'
                    assert 'nav_slots: 12-96:4' in ret.message, f'ERROR: nav_slots not expected range: 12-96:4, aloha_slots should trim nav_slots'
                    assert 'aloha_slots: 100-152:2,0-10:1,280-65534:1' in ret.message, f'ERROR: aloha_slots not expected range: 100-152:2,0-10:1,280-65534:1'
                else:
                    # assert str(expanded_slots_as_ranges(ret.nav_slots)) == '0-148:4', f'ERROR: nav_slots not expected range: 0-148:4'
                    assert 'nav_slots: 0-148:4' in ret.message, f'ERROR: nav_slots not expected range: 0-148:4'

                # assert str(expanded_slots_as_ranges(ret.comms_slots)) == '0-150:2', f'ERROR: comms_slots not expected range: 0-150:2'
                # assert str(expanded_slots_as_ranges(ret.active_slots)) == '0-150:2', f'ERROR: active_slots not expected range: 0-150:2'
                assert 'comms_slots: 0-150:2' in ret.message, f'ERROR: comms_slots not expected range: 0-150:2'
                assert 'active_slots: 0-150:2' in ret.message, f'ERROR: active_slots not expected range: 0-150:2'

            except rospy.exceptions.ROSException:
                self.logger.warning("timed out while waiting for tdma status")
                pytest.fail("timed out while waiting for tdma status")
            else:
                self.logger.warning("Testing Extreme Configs:: PASSED! slots in expected ranges")
                break

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """
        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
