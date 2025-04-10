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

class TestTdmaScripted:
    '''tdma Extended test class

    test cases:
        - test_cycling_rates_one_per_slot:... to pass, must have observed pattern 1, 3, 5 repeat back to back: [1, 3, 5, 1, 3, 5]
                                                * don't have a good way to be sure of the fix time we are txing so observed values might start,
                                                .. at 3 or 5 but must continue with the pattern to be valid
         ! N per slot tests are multiples of 3
         ! .. first packet in slot should be rate of first test plan value
        - test_cycling_rates_three_per_slot:. [1, 3, 5] all 3 rates go out in this modems active slot, only pass if [1, 3, 5], [1, 3, 5] two slots in a row
        - test_cycling_rates_six_per_slot:... [1, 3, 5, 1, 3, 5] all 3 rates go out then repeat in the same slot. only pass if [1, 3, 5, 1, 3, 5], [1, 3, 5, 1, 3, 5] two slots in a row
    '''
    VALID_TDMA_TYPES = ['tdma_scripted']
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """
        self.test_counter = 0
        rospy.init_node("test_tdma_scripted", log_level=rospy.DEBUG)
        self.logger = logging.getLogger("rosout")

        self.tdma_type = rospy.get_param("/tdma_type", "tdma_scripted")
        if self.tdma_type not in TestTdmaScripted.VALID_TDMA_TYPES:
            raise NotImplementedError(f'This test module does not work with {self.tdma_type}. Valid tdma_types: {TestTdmaScripted.VALID_TDMA_TYPES}')
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
                # active_slots_modem1='1',
                active_slots_modem2='0',     # Staggered double time modem2 active slots, 6 packets per slot
                active_slots_modem3='1,3,5,7', # Staggered half time modem3 active slots, 1 packet per slot
                tdma_type=self.tdma_type,
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
        self.logger.warning(f'test_tdma_scripted sleeping for one_cycle_duration ({one_cycle_duration} sec) before starting tests')
        rospy.sleep(one_cycle_duration)
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

    def test_cycling_rates_one_per_slot(self):
        self.logger.warning(f'Testing Cycling Rates on modem3 configured to trasmit 1 packet per slot')
        self.logger.info(f'Testing Cycling Rates 1 packet per slot:: Sleep for 2.0 then start test on /modem3.')
        rospy.sleep(2.0)

        max_cycled_rates_to_obs = 9 # max length of the list of rates transmitted before assertion test
        cycled_rates_obs = []       # dataframe_rates cycled

        while not rospy.is_shutdown():
            try:
                self.logger.info(f'Testing Cycling Rates 1 packet per slot:: Waiting for next message on /modem3/transmit. timeout={self.roslaunch_dict["slot_duration_seconds_half"]}')
                ret = rospy.wait_for_message(
                    "/modem3/transmit", TransmittedPacket, timeout=self.roslaunch_dict['slot_duration_seconds_half'])
            except rospy.exceptions.ROSException:
                self.logger.info(f'Testing Cycling Rates 1 packet per slot:: Timed-out waiting for /modem3/transmit. timeout={self.roslaunch_dict["slot_duration_seconds_half"]}')
            else:
                cycled_rates_obs.append(ret.packet.dataframe_rate)
                self.logger.warning(f'Test Cycling Rates 1 packet per slot:: Rate Cycled! rate cycles observed: {cycled_rates_obs}')
                if len(cycled_rates_obs) >= max_cycled_rates_to_obs:
                    self.logger.info(f'Test Cycling Rates 1 packet per slot:: Observed {max_cycled_rates_to_obs} cycles. cycled_rates_obs so far: {cycled_rates_obs}')
                    break

        # assert set(cycled_rates_obs).issubset({1, 3, 5, 1, 3, 5}), f'Error! Cycled rates (1 packet per slot) is not a subset of the test order: expected subset:{set([1, 3, 5, 1, 3, 5])}, observed:{set(cycled_rates_obs)}'
        assert '135135' in ''.join([str(v) for v in cycled_rates_obs]), f'Error! Cycled rates (1 packet per slot) is not a subset of the test order: expected subset: [1, 3, 5, 1, 3, 5] observed: {cycled_rates_obs}'
        assert '11' not in ''.join([str(v) for v in cycled_rates_obs]), f'Error! Cycled rates (1 packet per slot) subset present but rates are repeated! observed: {cycled_rates_obs}'
        assert '33' not in ''.join([str(v) for v in cycled_rates_obs]), f'Error! Cycled rates (1 packet per slot) subset present but rates are repeated! observed: {cycled_rates_obs}'
        assert '55' not in ''.join([str(v) for v in cycled_rates_obs]), f'Error! Cycled rates (1 packet per slot) subset present but rates are repeated! observed: {cycled_rates_obs}'

        self.logger.warning(f'Test Cycling Rates 1 packet per slot:: TEST PASSED; cycled_rates_obs: {cycled_rates_obs}, required subset: [1, 3, 5, 1, 3, 5]')

    def test_cycling_rates_three_per_slot(self):
        self.logger.warning(f'Testing Cycling Rates on modem0 configured to trasmit 3 packet per slot')
        self.logger.info(f'Testing Cycling Rates 3 packets per slot:: Sleep for first full cycle then start test on /modem0')
        test_tdma_scripted_sub = rospy.Subscriber(f'/modem0/tdma_scripted_status', TdmaScriptedStatus, callback=self._topic_callback)
        # rospy.sleep(self.one_cycle_duration)
        rospy.sleep(2.0)

        cycle_start_flag = False      # signal when we should start testing the packets in ~transmit. Start when we are not active so the sub is setup in time
        max_cycled_rates_to_obs = 6   # max length of the list of rates transmitted before assertion test
        set_start_t_this_slot = False # flag for setting time at start of active slot so time between packets can be recorded
        # max_cycled_rates_to_obs = 3 # 
        cycled_rates_obs = []         # dataframe_rates cycled
        cycled_rates_dt = []          # time delta in seconds between packets
                                      # .. the first time should be 0.0 then ~9.0-10.0 between subsequent packets in the same slot

        while not rospy.is_shutdown():
            if self.modem0_last is None:
                # wait until we have a status from this modem
                self.logger.info(f'Test Cycling Rates 3 packets per slot:: Waiting for right state on /modem0/tdma_scripted_status. Sleeping for 0.01...')
                rospy.sleep(0.01)
                continue

            # cache the value since this topic is updated freq
            modem0_last = self.modem0_last

            if not cycle_start_flag and modem0_last.current_slot not in [0, 2]:
                # wait until we're in an inactive slot so we can setup for the first packet in the next active slot
                cycle_start_flag = True

            if not cycle_start_flag:
                self.logger.debug(f'Test Cycling Rates 3 packets per slot:: Waiting until slot (1 or 3) after our active slot (0 or 2)')
                continue

            if not modem0_last.we_are_active:
                # set this whenever we are inactive so it's ready for next set of packets
                set_start_t_this_slot = False

            try:
                self.logger.info(f'Testing Cycling Rates 3 packets per slot:: Waiting for next message on /modem0/transmit. timeout=5.0')
                ret = rospy.wait_for_message(
                    "/modem0/transmit", TransmittedPacket, timeout=5.0)
            except rospy.exceptions.ROSException:
                self.logger.info(f'Testing Cycling Rates 3 packets per slot:: Timed-out waiting for /modem0/transmit. timeout=5.0')
            else:
                if not set_start_t_this_slot:
                    start_t = rospy.get_time()
                    set_start_t_this_slot = True

                cycled_rates_obs.append(ret.packet.dataframe_rate)
                # record time delta between cycled rate packets to be sure they are transmitted in the same slot
                cycled_rates_dt.append(rospy.get_time() - start_t)
                start_t = rospy.get_time()
                self.logger.warning(f'Test Cycling Rates 3 packets per slot:: Rate Cycled! rate cycles observed: {cycled_rates_obs}')
                if len(cycled_rates_obs) >= max_cycled_rates_to_obs:
                    self.logger.info(f'Test Cycling Rates 3 packets per slot:: Observed {max_cycled_rates_to_obs} cycles. cycled_rates_obs so far: {cycled_rates_obs}')
                    break

        # assert cycled_rates_obs == [1, 3, 5], f'Error! Cycled rates (3 packets per slot) are not in expected test order: expected:[1, 3, 5], observed:{cycled_rates_obs}, {cycled_rates_dt}'
        assert cycled_rates_obs == [1, 3, 5, 1, 3, 5], f'Error! Cycled rates (3 packets per slot) are not in expected test order: expected:[1, 3, 5, 1, 3, 5], observed:{cycled_rates_obs}'
        assert all([int(delta_t) <= 10 for delta_t in cycled_rates_dt]), f'Error! Cycled rates (3 packets per slot) has correct rates in order but the slot is aliased! Time deltas should be <= 10 sec {cycled_rates_dt}'

        self.logger.warning(f'Test Cycling Rates 3 packets per slot:: TEST PASSED')
        test_tdma_scripted_sub.unregister()

    def test_cycling_rates_six_per_slot(self):
        self.logger.warning(f'Testing Cycling Rates on modem2 configured to trasmit 6 packet per slot')
        self.logger.info(f'Testing Cycling Rates 6 packets per slot:: Sleep for full cycle duration then start test on /modem2.')
        test_tdma_scripted_sub = rospy.Subscriber(f'/modem2/tdma_scripted_status', TdmaScriptedStatus, callback=self._topic_callback)
        # rospy.sleep(self.one_cycle_duration)
        rospy.sleep(2.0)

        cycle_start_flag = False      # signal when we should start testing the packets in ~transmit. Start when we are not active so the sub is setup in time
        max_cycled_rates_to_obs = 12  # max length of the list of rates transmitted before assertion test
        set_start_t_this_slot = False # flag for setting time at start of active slot so time between packets can be recorded
        # max_cycled_rates_to_obs = 3 # 
        cycled_rates_obs = []         # dataframe_rates cycled
        cycled_rates_dt = []          # time delta in seconds between packets
                                      # .. the first time should be 0.0 then ~9.0-10.0 between subsequent packets in the same slot

        while not rospy.is_shutdown():
            if self.modem2_last is None:
                # wait until we have a status from this modem
                self.logger.info(f'Test Cycling Rates 6 packets per slot:: Waiting for right state on /modem2/tdma_scripted_status. Sleeping for 0.01...')
                rospy.sleep(0.01)
                continue

            # cache the value since this topic is updated freq
            modem2_last = self.modem2_last

            if not cycle_start_flag and modem2_last.current_slot == 1:
                # wait until we're in an inactive slot so we can setup for the first packet in the next active slot
                cycle_start_flag = True

            if not cycle_start_flag:
                self.logger.debug(f'Test Cycling Rates 6 packets per slot:: Waiting until slot (1) after our active slot (0).')
                # still haven't gotten to a safe starting slot...
                continue

            if not modem2_last.we_are_active:
                # set this whenever we are inactive so it's ready for next set of packets
                set_start_t_this_slot = False

            try:
                self.logger.info(f'Testing Cycling Rates 3 packets per slot:: Waiting for next message on /modem2/transmit. timeout=5.0')
                ret = rospy.wait_for_message(
                    "/modem2/transmit", TransmittedPacket, timeout=5.0)
            except rospy.exceptions.ROSException:
                self.logger.info(f'Testing Cycling Rates 3 packets per slot:: Timed-out waiting for /modem2/transmit. timeout=5.0')
            else:
                if not set_start_t_this_slot:
                    start_t = rospy.get_time()
                    set_start_t_this_slot = True

                cycled_rates_obs.append(ret.packet.dataframe_rate)
                # record time delta between cycled rate packets to be sure they are transmitted in the same slot
                cycled_rates_dt.append(rospy.get_time() - start_t)
                start_t = rospy.get_time()
                self.logger.warning(f'Test Cycling Rates 3 packets per slot:: Rate Cycled! rate cycles observed: {cycled_rates_obs}')
                if len(cycled_rates_obs) >= max_cycled_rates_to_obs:
                    self.logger.info(f'Test Cycling Rates 3 packets per slot:: Observed {max_cycled_rates_to_obs} cycles. cycled_rates_obs so far: {cycled_rates_obs}')
                    break

        assert cycled_rates_obs == [1, 3, 5, 1, 3, 5, 1, 3, 5, 1, 3, 5], f'Error! Cycled rates (6 packets per slot) are not in expected test order: expected:[1, 3, 5, 1, 3, 5], observed:{cycled_rates_obs}'
        assert all([int(delta_t) <= 10 for delta_t in cycled_rates_dt]), f'Error! Cycled rates (6 packets per slot) has correct rates in order but the slot is aliased! Time deltas should be <= 10 sec {cycled_rates_dt}'

        self.logger.warning(f'Test Cycling Rates 6 packets per slot:: TEST PASSED')
        self.logger.warning(f'Test Cycling Rates 6 packets per slot:: cycled_rates_dt:.. {cycled_rates_dt}')
        self.logger.warning(f'Test Cycling Rates 6 packets per slot:: cycled_rates_obs:. {cycled_rates_obs}')
        self.logger.warning(f'Test Cycling Rates 6 packets per slot:: expected:......... [1, 3, 5, 1, 3, 5, 1, 3, 5, 1, 3, 5]')
        test_tdma_scripted_sub.unregister()

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """
        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
