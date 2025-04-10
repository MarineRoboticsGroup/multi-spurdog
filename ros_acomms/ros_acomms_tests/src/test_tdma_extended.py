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
from ros_acomms_msgs.msg import TdmaAdvancedStatus, TdmaScriptedStatus, TdmaSlottedAlohaStatus, ReceivedPacket
from ros_acomms_tests.msg import FragmentationTestMsg

class TestTdmaExtended:
    '''tdma Extended test class

    test cases:
    - manual_transmit_queue
    - manual_transmit_queue, with always_send_test_data
    - software_mute
        - TODO: software_mute, with manual_transmit_queue
        - TODO: software_mute, with always_send_test_data
    - always_send_test_data
    - always_send_test_data, with miniframe data
    - always_send_test_data, with dataframe data
    '''
    VALID_TDMA_TYPES = ['tdma_advanced', 'tdma_scripted', 'tdma_slotted_aloha']
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """
        self.test_counter = 0
        rospy.init_node("test_tdma_extended", log_level=rospy.DEBUG)
        self.logger = logging.getLogger("rosout")

        self.tdma_type = rospy.get_param("/tdma_type", "tdma_scripted")
        if self.tdma_type not in TestTdmaExtended.VALID_TDMA_TYPES:
            raise NotImplementedError(f'This test module does not work with {self.tdma_type}. Valid tdma_types: {TestTdmaExtended.VALID_TDMA_TYPES}')
        if self.tdma_type == 'tdma_advanced':
            self.tdma_msg_type = TdmaAdvancedStatus
        elif self.tdma_type == 'tdma_scripted':
            self.tdma_msg_type = TdmaScriptedStatus
        elif self.tdma_type == 'tdma_slotted_aloha':
            self.tdma_msg_type = TdmaSlottedAlohaStatus

        self.roslaunch_dict = dict(
                num_slots=4,
                slot_duration_seconds=15,
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
            "launch/test_tdma_extended.launch",
        )

        roslaunch_args = [f"{k}:={v}" for k, v in self.roslaunch_dict.items()]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()
        time.sleep(5)
        self.logger.info('Node: test_tdma_extended running!')

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

    def test_always_send_test_data(self):
        '''test_always_send_test_data

        - wait for next message from src:3 on /modem0/packet_rx and if all 0's, pass
        '''
        self.logger.info(f'Testing Always Send Test Data')
        max_timed_out_attempts = self.roslaunch_dict['num_slots'] + 1
        num_timed_out = 0

        while True:
            try:
                self.logger.info(f'Always Send Test Data:: Waiting for msg on /modem0/packet_rx, timeout=20')
                ret = rospy.wait_for_message(
                    "/modem0/packet_rx", ReceivedPacket, timeout=20)
                if ret.packet.src == 3:
                    assert any(ret.packet.miniframe_bytes) == False, f'Packet from modem3 is not all zeros for miniframe_bytes {ret.packet.miniframe_bytes}'
                    assert any(ret.packet.dataframe_bytes) == False, f'Packet from modem3 is not all zeros for dataframe_bytes {ret.packet.dataframe_bytes}'
                    self.logger.info(f'Always Send Test Data:: Got blank packet from modem3! {ret.packet} ')
                    break
            except rospy.exceptions.ROSException:
                num_timed_out += 1
                self.logger.info(f'Always Send Test Data:: Timed-out waiting for msg on /modem0/packet_rx')
                if num_timed_out >= max_timed_out_attempts:
                    pytest.fail("reached max_timed_out_attempts when waiting for a message on packet_rx")
        self.logger.info(f'Always Send Test Data:: TEST PASSED')

    def test_always_send_test_data_with_dataframes(self):
        '''test_always_send_test_data_with_dataframes

        - publish to topic on modem3 so miniframes will overflow into dataframes
        - wait for next message from src:3 on /modem0/packet_rx make sure it has 0's at the end and some data inside dataframe_bytes
        '''
        self.logger.info(f'Testing Always Send Test Data data in dataframes')
        max_timed_out_attempts = 2
        num_timed_out = 0
        queue_publisher = rospy.Publisher('/modem3/test_fragmentation_msg', FragmentationTestMsg, queue_size=5)
        time.sleep(2)

        msg = FragmentationTestMsg()
        msg.data = secrets.token_bytes(192)
        queue_publisher.publish(msg)

        while True:
            try:
                self.logger.info(f'Always Send Test Data dataframes:: Waiting for msg on /modem0/packet_rx, timeout=100')
                ret = rospy.wait_for_message(
                    "/modem0/packet_rx", ReceivedPacket, timeout=100)
                if ret.packet.src == 3:
                    dataframe_byte_offset = len(ret.packet.dataframe_bytes) - 124
                    self.logger.info(f'Always Send Test Data dataframes:: dataframe_byte_offset: {dataframe_byte_offset} ')
                    assert any(ret.packet.miniframe_bytes), f'Packet from modem3 is all zeros for miniframe_bytes {ret.packet.miniframe_bytes}. Published test_msg not transmitted'
                    assert ret.packet.dataframe_bytes[-dataframe_byte_offset:] == bytes([0] * dataframe_byte_offset), f'Packet from modem3 should have {dataframe_byte_offset} zero padded bits at the end {ret.packet.dataframe_bytes}.'
                    self.logger.info(f'Always Send Test Data dataframes:: Got valid zero padded packet with some miniframe bytes from modem3! {ret.packet} ')
                    break
            except rospy.exceptions.ROSException:
                num_timed_out += 1
                self.logger.info(f'Always Send Test Data dataframes:: Timed-out waiting for msg on /modem0/packet_rx')
                if num_timed_out >= max_timed_out_attempts:
                    pytest.fail("reached max_timed_out_attempts when waiting for a message on packet_rx")
        self.logger.info(f'Always Send Test Data dataframes:: TEST PASSED')

    def test_software_mute(self):
        '''test_software_mute

        - publish msg on modem2/test_msg (this modem is software_muted)
        - wait for next message to timeout on modem3/from_acomms/test_msg
        '''
        self.logger.info(f'Testing Software Mute')
        queue_publisher = rospy.Publisher('/modem2/test_msg', UInt8, queue_size=5)
        msg = UInt8()
        msg.data = 123
        queue_publisher.publish(msg)
        max_timed_out_attempts = 2
        num_timed_out = 0

        while True:
            try:
                self.logger.info(f'Software Mute:: Waiting for msg on /modem3/from_acomms/test_msg, timeout=20')
                self.logger.info(f'Software Mute:: Attempt {num_timed_out} of {max_timed_out_attempts}')
                ret = rospy.wait_for_message(
                    "/modem3/from_acomms/test_msg", UInt8, timeout=20)
                assert ret.data != 123, f'Software mute failed on modem2, test_msg: {ret} was still transmitted!'
            except rospy.exceptions.ROSException:
                num_timed_out += 1
                self.logger.info(f'Software Mute:: Timed-out waiting for txd msg from modem2 on /modem3/from_acomms/test_msg, this is expected since modem2 is software_muted')
                if num_timed_out >= max_timed_out_attempts:
                    break # passed
        self.logger.info(f'Software Mute:: TEST PASSED')

    def test_always_send_test_data_with_miniframes(self):
        '''test_always_send_test_data_with_miniframes

        - publish to topic on modem3 so miniframes will be populated
        - wait for next message from src:3 on /modem0/packet_rx make sure it has 0's at the end and some data inside miniframes
        '''
        self.logger.info(f'Testing Always Send Test Data data in miniframes')
        max_timed_out_attempts = 2
        num_timed_out = 0
        # sometimes we are so fast we get the last packet sent with all zeros. Only allow this on the first
        got_prior_packet = False
        queue_publisher = rospy.Publisher('/modem3/test_msg', UInt8, queue_size=5)
        time.sleep(2)

        msg = UInt8()

        for val in range(3):
            msg.data = val
            queue_publisher.publish(msg)
            self.logger.info(f'Always Send Test Data miniframes:: published msg: {msg} on /modem3/test_msg')

        while True:
            try:
                self.logger.info(f'Always Send Test Data miniframes:: Waiting for msg on /modem0/packet_rx, timeout=100')
                ret = rospy.wait_for_message(
                    "/modem0/packet_rx", ReceivedPacket, timeout=100)
                if ret.packet.src == 3:
                    try:
                        assert any(ret.packet.miniframe_bytes), f'Packet from modem3 is all zeros for miniframe_bytes {ret.packet.miniframe_bytes}. Published test_msg not transmitted'
                        assert ret.packet.miniframe_bytes[-64:] == bytes([0] * 64), f'Packet from modem3 should have 64 zero padded bits at the end {ret.packet.miniframe_bytes}.'
                    except AssertionError:
                        # if we have not already gotten a prior message, re-raise
                        if not got_prior_packet and num_timed_out == 0:
                            # we allow 1 old packet so we can avoid watching tdma status
                            got_prior_packet = True
                            self.logger.info(f'Always Send Test Data miniframes:: got prior packet from modem3.. allowing once.')
                            continue
                        else:
                            raise
                    else:
                        self.logger.info(f'Always Send Test Data miniframes:: Got valid zero padded packet with 64 zero padded bytes at end of miniframe_bytes from modem3! {ret.packet} ')
                        break
            except rospy.exceptions.ROSException:
                num_timed_out += 1
                self.logger.info(f'Always Send Test Data miniframes:: Timed-out waiting for msg on /modem0/packet_rx')
                if num_timed_out >= max_timed_out_attempts:
                    pytest.fail("reached max_timed_out_attempts when waiting for a message on packet_rx")
        self.logger.info(f'Always Send Test Data miniframes:: TEST PASSED')

    def test_manual_transmit(self):
        '''test_manual_transmit

        - makes sure we're not active on modem0 and we're on the slot before our active slot
        - queue message on /modem0/tdma/nmea_to_modem
        - setup subscriber for /modem0/{self.tdma_type}_status
        - wait for message on /modem0/nmea_to_modem
        - when we get message on /modem0/nmea_to_modem, check last tdma_status for pass criteria
        '''
        self.logger.info(f'Testing Manual Transmit')
        queue_publisher = rospy.Publisher('/modem0/tdma/nmea_to_modem', String, queue_size=5)
        test_manual_transmit_sub = rospy.Subscriber(f'/modem0/{self.tdma_type}_status', self.tdma_msg_type, callback=self._topic_callback)
        time.sleep(2)

        # wait until we are at slot 1, (one past our active slot for modem0)
        while True:
            if self.modem0_last is None:
                self.logger.info(f'Manual Transmit:: Waiting for right state on /modem0/{self.tdma_type}_status. Sleeping for 1.0...')
                rospy.sleep(1.0)
                continue
            if self.modem0_last.we_are_active:
                rospy.sleep(self.modem0_last.remaining_slot_seconds)
                continue

            if self.modem0_last.current_slot == self.roslaunch_dict['num_slots'] - 1:
                self.logger.info(f'Manual Transmit:: Got tdma_status and we are in slot before our active slot, queue manual_transmit messages')
                break
            else:
                rospy.sleep(self.modem0_last.remaining_slot_seconds)
                continue

        msg = String()
        msg.data = '$CCACM,1,1,1,CCCFQ;pwramp.txlevel'
        queue_publisher.publish(msg)

        while True:
            try:
                self.logger.info(f'Manual Transmit:: Waiting for msg on /modem0/nmea_to_modem, timeout={self.roslaunch_dict["slot_duration_seconds"] + 5}')
                ret = rospy.wait_for_message("/modem0/nmea_to_modem", String, timeout=self.roslaunch_dict['slot_duration_seconds'] + 5)
                modem0_last = self.modem0_last
                assert ret.data == msg.data, f'Last message on /modem0/nmea_to_modem is not value sent: rx:{ret.data} != tx:{msg.data}'
                assert modem0_last.we_are_active, f'Manual transmit happened while we were not active...'
                self.logger.info(f'Manual Transmit:: Last message on /modem0/nmea_to_modem is value sent: rx:{ret.data} == tx:{msg.data}')
                break
            except rospy.exceptions.ROSException:
                pytest.fail(f"Timed-out before getting manual_transmit!")

        test_manual_transmit_sub.unregister()
        self.logger.info(f'Manual Transmit:: TEST PASSED')

    def test_manual_transmit_with_always_send_test_data(self):
        '''test_manual_transmit_with_always_send_test_data

        - makes sure we're not active on modem3 and we're on the slot before our active slot
        - queue message on /modem0/tdma/nmea_to_modem
        - setup subscriber for /modem3/{self.tdma_type}_status
        - wait for message on /modem3/nmea_to_modem
        - when we get message on /modem3/nmea_to_modem, check last tdma_status for pass criteria
        '''
        self.logger.info(f'Testing Manual Transmit with Always Send Test Data')
        queue_publisher = rospy.Publisher('/modem3/tdma/nmea_to_modem', String, queue_size=5)
        test_manual_transmit_sub = rospy.Subscriber(f'/modem3/{self.tdma_type}_status', self.tdma_msg_type, callback=self._topic_callback)
        time.sleep(2)

        # wait until we are at slot 1, (one past our active slot for modem0)
        while True:
            if self.modem3_last is None:
                self.logger.info(f'Manual Transmit w ASTD:: Waiting for right state on /modem3/{self.tdma_type}_status. Sleeping for 1.0...')
                rospy.sleep(1.0)
                continue
            if self.modem3_last.we_are_active:
                rospy.sleep(self.modem3_last.remaining_slot_seconds)
                continue

            if self.modem3_last.current_slot == 2:
                self.logger.info(f'Manual Transmit w ASTD:: Got tdma_status and we are in slot before our active slot, queue manual_transmit messages')
                break
            else:
                rospy.sleep(self.modem3_last.remaining_slot_seconds)
                continue

        msg = String()
        msg.data = '$CCACM,0,1,1,CCCFQ;pwramp.txlevel'
        queue_publisher.publish(msg)

        while True:
            try:
                self.logger.info(f'Manual Transmit w ASTD:: Waiting for msg on /modem3/nmea_to_modem, timeout={self.roslaunch_dict["slot_duration_seconds"] + 5}')
                ret = rospy.wait_for_message("/modem3/nmea_to_modem", String, timeout=self.roslaunch_dict['slot_duration_seconds'] + 5)
                modem3_last = self.modem3_last
                assert ret.data == msg.data, f'Last message on /modem3/nmea_to_modem is not value sent: rx:{ret.data} != tx:{msg.data}'
                assert modem3_last.we_are_active, f'Manual transmit happened while we were not active...'
                self.logger.info(f'Manual Transmit w ASTD:: Last message on /modem3/nmea_to_modem is value sent: rx:{ret.data} == tx:{msg.data}')
                break
            except rospy.exceptions.ROSException:
                pytest.fail(f"Timed-out before getting manual_transmit!")

        test_manual_transmit_sub.unregister()
        self.logger.info(f'Manual Transmit w ASTD:: TEST PASSED')

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """
        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
