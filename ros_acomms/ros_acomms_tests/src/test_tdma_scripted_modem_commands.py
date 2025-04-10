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
from ros_acomms_msgs.msg import TdmaScriptedStatus, ReceivedPacket, TransmittedPacket
from ros_acomms_tests.msg import FragmentationTestMsg

class TestTdmaScriptedModemCommands:
    '''tdma Extended test class

    test cases:
        - test_modem_cmds_one_packet_per_slot:... to pass, must have observed pattern 1, 3, 5 repeat back to back: [1, 3, 5, 1, 3, 5]
                                                * don't have a good way to be sure of the fix time we are txing so observed values might start,
                                                .. at 3 or 5 but must continue with the pattern to be valid
         ! N per slot tests are multiples of 3
         ! .. first packet in slot should be rate of first test plan value
        - test_modem_cmds_three_packets_per_slot:. [1, 3, 5] all 3 rates go out in this modems active slot, only pass if [1, 3, 5], [1, 3, 5] two slots in a row
        - test_modem_cmds_six_packets_per_slot:... [1, 3, 5, 1, 3, 5] all 3 rates go out then repeat in the same slot. only pass if [1, 3, 5, 1, 3, 5], [1, 3, 5, 1, 3, 5] two slots in a row
    '''
    VALID_TDMA_TYPES = ['tdma_scripted']
    EXPECTED_MODEM_CMD_ORDER = [
        '$CCCFQ,SRC\r\n',
        # '$CCCFG,pwramp.txlevel,1\r\n',
        '$CCCFQ,AGN\r\n',
        # '$CCCFG,pwramp.txlevel,2\r\n',
        '$CCCFQ,DTH\r\n',
        # '$CCCFG,pwramp.txlevel,3\r\n',
    ]
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """
        self.test_counter = 0
        rospy.init_node("test_tdma_scripted_modem_commands", log_level=rospy.DEBUG)
        self.logger = logging.getLogger("rosout")

        self.tdma_type = rospy.get_param("/tdma_type", "tdma_scripted")
        if self.tdma_type not in TestTdmaScriptedModemCommands.VALID_TDMA_TYPES:
            raise NotImplementedError(f'This test module does not work with {self.tdma_type}. Valid tdma_types: {TestTdmaScriptedModemCommands.VALID_TDMA_TYPES}')
        if self.tdma_type == 'tdma_scripted':
            self.tdma_msg_type = TdmaScriptedStatus

        self.roslaunch_dict = dict(
                num_slots=4,
                slot_duration_seconds=30,
                num_slots_half=8,
                slot_duration_seconds_half=15,
                num_slots_double=2,
                slot_duration_seconds_double=60,
                active_slots_modem0='0,2',   # Staggered 3 per slot tested with modem0
                active_slots_modem2='0',     # Staggered double time modem2 active slots, 6 packets per slot
                active_slots_modem3='1,3,5,7', # Staggered half time modem3 active slots, 1 packet per slot
                tdma_type=self.tdma_type,
                test_cycling_rates=False,
            )

        self.modem0_last = None
        self.modem1_last = None
        self.modem2_last = None
        self.modem3_last = None

        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/test_tdma_scripted.launch",
        )

        roslaunch_args = [f"{k}:={v}" for k, v in self.roslaunch_dict.items()]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()

        one_cycle_duration = self.roslaunch_dict['slot_duration_seconds'] * self.roslaunch_dict['num_slots']
        one_cycle_duration_half = self.roslaunch_dict['slot_duration_seconds_half'] * self.roslaunch_dict['num_slots_half']
        one_cycle_duration_double = self.roslaunch_dict['slot_duration_seconds_double'] * self.roslaunch_dict['num_slots_double']
        assert one_cycle_duration == one_cycle_duration_half == one_cycle_duration_double
        self.one_cycle_duration = one_cycle_duration
        self.logger.warning(f'test_tdma_scripted_modem_commands sleeping for one_cycle_duration ({one_cycle_duration / 2} sec) before starting tests')
        rospy.sleep(one_cycle_duration / 2)
        self.logger.warning('Node: test_tdma_extended running!')

    def _topic_callback(self, msg):
        topic = msg._connection_header['topic']
        if 'modem0' in topic:
            self.modem0_last = msg
        elif 'modem1' in topic:
            self.modem1_last = msg
        elif 'modem2' in topic:
            self.modem2_last = msg
        elif 'modem3' in topic:
            self.modem3_last = msg

    def test_modem_cmds_one_packet_per_slot(self):
        self.logger.warning(f'Testing Modem Commands on modem3 configured to trasmit 1 packet per slot')
        self.logger.info(f'Testing Modem Commands 1 packet per slot, 1 modem commands per packet:: Sleep for {self.roslaunch_dict["slot_duration_seconds_half"]} then start test on /modem3.')
        test_tdma_scripted_modem_commands_sub = rospy.Subscriber(f'/modem3/tdma_scripted_status', TdmaScriptedStatus, callback=self._topic_callback)
        rospy.sleep(self.roslaunch_dict["slot_duration_seconds_half"])

        while not rospy.is_shutdown():
            if self.modem3_last is None:
                # wait until we have a status from this modem
                self.logger.info(f'Test Modem Commands 1 packet per slot, 1 modem commands per packet:: Waiting for right state on /modem3/tdma_scripted_status. Sleeping for 0.1...')
                rospy.sleep(0.1)
                continue

            modem3_last = self.modem3_last
            if not modem3_last.we_are_active:
                rospy.sleep(modem3_last.remaining_slot_seconds - 5.0)
                break

        max_modem_cmds_to_obs = 9 # max length of the list of rates transmitted before assertion test
        nmea_to_modem_obs = []    # nmea_to_modem commands

        while not rospy.is_shutdown():
            try:
                self.logger.info(f'Testing Modem Commands 1 packet per slot:: Waiting for next message on /modem3/nmea_to_modem. timeout={self.roslaunch_dict["slot_duration_seconds_half"]}')
                query_cmd, pwr_command = None, None
                query_cmd = rospy.wait_for_message(
                    "/modem3/nmea_to_modem", String, timeout=self.roslaunch_dict['slot_duration_seconds_half'])
            except rospy.exceptions.ROSException:
                self.logger.info(f'Testing Modem Commands 1 packet per slot:: Timed-out waiting for /modem3/nmea_to_modem. timeout={self.roslaunch_dict["slot_duration_seconds_half"]}')
                # self.logger.info(f'Testing Modem Commands 1 packet per slot:: Timed-out waiting for /modem3/nmea_to_modem. query_cmd={query_cmd}, pwr_command={pwr_command}')
                self.logger.info(f'Testing Modem Commands 1 packet per slot:: Timed-out waiting for /modem3/nmea_to_modem. query_cmd={query_cmd}')
            else:
                nmea_to_modem_obs.append(query_cmd.data)
                # nmea_to_modem_obs.append(pwr_command.data)
                self.logger.warning(f'Test Modem Commands 1 packet per slot:: 1 modem commands recvd! modem commands observed: {nmea_to_modem_obs}')
                if len(nmea_to_modem_obs) >= max_modem_cmds_to_obs:
                    self.logger.info(f'Test Modem Commands 1 packet per slot:: max {max_modem_cmds_to_obs}. modem commands so far: {nmea_to_modem_obs}')
                    break
                else:
                    rospy.sleep(self.modem3_last.remaining_slot_seconds + 5)
                    modem3_last = self.modem3_last
                    if not modem3_last.we_are_active:
                        rospy.sleep(modem3_last.remaining_slot_seconds - 5.0)

        test_string = ''.join([str(v) for v in nmea_to_modem_obs])
        assert ''.join(TestTdmaScriptedModemCommands.EXPECTED_MODEM_CMD_ORDER) in test_string, f'Error! Modem Commands (1 packet per slot) is not a subset of the test order: expected subset: {TestTdmaScriptedModemCommands.EXPECTED_MODEM_CMD_ORDER} observed: {nmea_to_modem_obs}'
        self.logger.warning(f'Test Modem Commands 1 packet per slot:: TEST PASSED; modem commands observed: {nmea_to_modem_obs}, required subset: {TestTdmaScriptedModemCommands.EXPECTED_MODEM_CMD_ORDER}')
        test_tdma_scripted_modem_commands_sub.unregister()

    # def test_modem_cmds_one_packet_per_slot(self):
    #     self.logger.warning(f'Testing Modem Commands on modem3 configured to trasmit 1 packet per slot')
    #     self.logger.info(f'Testing Modem Commands 1 packet per slot, 2 modem commands per packet:: Sleep for {self.roslaunch_dict["slot_duration_seconds_half"]} then start test on /modem3.')
    #     test_tdma_scripted_modem_commands_sub = rospy.Subscriber(f'/modem3/tdma_scripted_status', TdmaScriptedStatus, callback=self._topic_callback)
    #     rospy.sleep(self.roslaunch_dict["slot_duration_seconds_half"])

    #     while not rospy.is_shutdown():
    #         if self.modem3_last is None:
    #             # wait until we have a status from this modem
    #             self.logger.info(f'Test Modem Commands 1 packet per slot, 2 modem commands per packet:: Waiting for right state on /modem3/tdma_scripted_status. Sleeping for 0.1...')
    #             rospy.sleep(0.1)
    #             continue

    #         modem3_last = self.modem3_last
    #         if not modem3_last.we_are_active:
    #             rospy.sleep(modem3_last.remaining_slot_seconds - 5.0)
    #             break

    #     max_modem_cmds_to_obs = 12 # max length of the list of rates transmitted before assertion test
    #     nmea_to_modem_obs = []     # nmea_to_modem commands

    #     while not rospy.is_shutdown():
    #         try:
    #             self.logger.info(f'Testing Modem Commands 1 packet per slot:: Waiting for next message on /modem3/nmea_to_modem. timeout={self.roslaunch_dict["slot_duration_seconds_half"]}')
    #             query_cmd, pwr_command = None, None
    #             query_cmd = rospy.wait_for_message(
    #                 "/modem3/nmea_to_modem", String, timeout=self.roslaunch_dict['slot_duration_seconds_half'])
    #             rospy.sleep(1.0)
    #             pwr_command = rospy.wait_for_message(
    #                 "/modem3/nmea_to_modem", String, timeout=0.5)
    #                 # "/modem3/nmea_to_modem", String, timeout=self.roslaunch_dict['slot_duration_seconds_half'])
    #         except rospy.exceptions.ROSException:
    #             self.logger.info(f'Testing Modem Commands 1 packet per slot:: Timed-out waiting for /modem3/nmea_to_modem. timeout={self.roslaunch_dict["slot_duration_seconds_half"]}')
    #             self.logger.info(f'Testing Modem Commands 1 packet per slot:: Timed-out waiting for /modem3/nmea_to_modem. query_cmd={query_cmd}, pwr_command={pwr_command}')
    #         else:
    #             nmea_to_modem_obs.append(query_cmd.data)
    #             nmea_to_modem_obs.append(pwr_command.data)
    #             self.logger.warning(f'Test Modem Commands 1 packet per slot:: 2 modem commands recvd! modem commands observed: {nmea_to_modem_obs}')
    #             if len(nmea_to_modem_obs) >= max_modem_cmds_to_obs:
    #                 self.logger.info(f'Test Modem Commands 1 packet per slot:: max {max_modem_cmds_to_obs}. modem commands so far: {nmea_to_modem_obs}')
    #                 break

    #     test_string = ''.join([str(v) for v in nmea_to_modem_obs])
    #     assert ''.join(TestTdmaScriptedModemCommands.EXPECTED_MODEM_CMD_ORDER) in test_string, f'Error! Modem Commands (1 packet per slot) is not a subset of the test order: expected subset: {TestTdmaScriptedModemCommands.EXPECTED_MODEM_CMD_ORDER} observed: {nmea_to_modem_obs}'
    #     self.logger.warning(f'Test Modem Commands 1 packet per slot:: TEST PASSED; modem commands observed: {nmea_to_modem_obs}, required subset: {TestTdmaScriptedModemCommands.EXPECTED_MODEM_CMD_ORDER}')
    #     test_tdma_scripted_modem_commands_sub.unregister()

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """
        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
