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

from std_msgs.msg import UInt8
from ros_acomms_msgs.msg import TdmaSlottedAlohaStatus

class TestTdmaSlottedAloha:
    '''tdma_slotted_aloha test class
    HighPass will test case where publishing a low priority message with no non-aloha slots means never tx
    LowPass will test case where only high priority messages are able to be sent
    Collision will test case where we have two nodes with high priority messages transmitting in the same aloha slot
    Low Priority will test case where low priority is sent during next non-aloha slot
    '''
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """
        self.test_counter = 0
        rospy.init_node("test_tdma_slotted_aloha", log_level=rospy.DEBUG)
        self.logger = logging.getLogger("rosout")

        self.roslaunch_dict = dict(
                num_slots=6,
                slot_duration_seconds=15,
                active_slots_high_pass="0,1",
                aloha_slots_high_pass="0,1,3", # intentional ununsed slot 2
                active_slots_low_pass="4,5",
                aloha_slots_low_pass="3",
                aloha_slot_priority=75,
            )

        self.modem0_last = None
        self.modem1_last = None

        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/test_slotted_aloha.launch",
        )

        roslaunch_args = [f"{k}:={v}" for k, v in self.roslaunch_dict.items()]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()
        time.sleep(5)
        self.logger.info('Node: test_tdma_slotted_aloha running!')

    def _topic_callback(self, msg):
        topic = msg._connection_header['topic']
        last_record = {'msg': msg, 'time': rospy.get_time()}
        if 'modem0' in topic:
            self.modem0_last = last_record
        elif 'modem1' in topic:
            self.modem1_last = last_record

    def test_high_pass(self):
        '''test_high_pass

        publishes 3 msgs (on low priority topic) to a node with only aloha slots so it must timeout to pass
        '''
        self.logger.info(f'Testing High Pass')
        queue_publisher = rospy.Publisher('/modem0/test_msg', UInt8, queue_size=5)
        time.sleep(2)

        return_arr = []
        input_arr = []

        msg = UInt8()

        for val in range(3):
            msg.data = val
            queue_publisher.publish(msg) # fire off all 3

        for val in range(3): # then wait for each
            try:
                self.logger.info(f'High Pass:: Waiting for timeout on /modem1/from_acomms/test_msg, timeout=50')
                ret = rospy.wait_for_message(
                    "/modem1/from_acomms/test_msg", UInt8, timeout=50)
            except Exception as e:
                assert str(type(e)) != 'rospy.exceptions.ROSException', f'Unexpected exception: {e}'
            else:
                pytest.fail(f"low priority message transmitted during aloha slot! {ret}")
        self.logger.info(f'High Pass:: TEST PASSED')

    def test_low_pass(self):
        '''test_low_pass

        - publishes 3 msgs on high priority topic
        '''
        self.logger.info(f'Testing Low Pass')
        queue_publisher = rospy.Publisher('/modem0/test_msg_aloha', UInt8, queue_size=5)
        time.sleep(2)

        msg = UInt8()
        msg.data = 255
        queue_publisher.publish(msg)

        try:
            self.logger.info(f'Low Pass:: Waiting for msg on /modem1/from_acomms/test_msg_aloha, timeout=50')
            ret = rospy.wait_for_message(
                "/modem1/from_acomms/test_msg_aloha", UInt8, timeout=50)
            assert ret.data == 255, f'Got the wrong test_msg_aloha. Expected 255 got {msg.data}'
        except rospy.exceptions.ROSException:
            pytest.fail(f"Timed-out before getting aloha message!")
        self.logger.info(f'Low Pass:: TEST PASSED')

    def test_collision(self):
        '''test_collision

        - makes sure we're not active and at least 1 slot away from shared aloha_slot
        - publishes 1 msg on modem0 high priority topic
        - publishes 1 msg on modem1 high priority topic
        '''
        self.logger.info(f'Testing Collision')
        queue_publisher_modem0 = rospy.Publisher('/modem0/test_msg_aloha', UInt8, queue_size=5)
        queue_publisher_modem1 = rospy.Publisher('/modem1/test_msg_aloha', UInt8, queue_size=5)
        time.sleep(2)

        # wait until we are at slot 2, then queue collision packets to go out same time during shared slot 3
        while True:
            msg = rospy.wait_for_message("/modem0/tdma_slotted_aloha_status", TdmaSlottedAlohaStatus, timeout=10)
            if msg.we_are_active:
                rospy.sleep(msg.remaining_slot_seconds)
                continue

            if msg.current_slot == 2:
                self.logger.info(f'Collision:: Got tdma_status from /modem0/tdma_slotted_aloha_status. We are in slot 2, queue collision packets..')
                break
            else:
                rospy.sleep(msg.remaining_slot_seconds)
                continue

        msg = UInt8()
        msg.data = 100
        queue_publisher_modem0.publish(msg)
        msg.data = 101
        queue_publisher_modem1.publish(msg)

        rospy.Subscriber('/modem0/from_acomms/test_msg_aloha', UInt8, callback=self._topic_callback)
        rospy.Subscriber('/modem1/from_acomms/test_msg_aloha', UInt8, callback=self._topic_callback)

        while True:
            if self.modem0_last is None or self.modem1_last is None:
                self.logger.info(f'Collision:: Waiting for msg on /modem0/from_acomms/test_msg_aloha AND, /modem1/from_acomms/test_msg_aloha. Sleeping for 10...')
                rospy.sleep(10.0)
                continue
            else:
                self.logger.info(f'Collision:: Got last message for modem0 and modem1')
                self.logger.info(f'Collision::  - self.modem0_last: {self.modem0_last}')
                self.logger.info(f'Collision::  - self.modem1_last: {self.modem1_last}')
                assert abs(self.modem0_last['time'] - self.modem1_last['time']) < 5, f'No collision in aloha slot!'
                assert self.modem0_last['msg'].data == 101, f'Wrong packet pair for collision'
                assert self.modem1_last['msg'].data == 100, f'Wrong packet pair for collision'
                self.logger.info(f"Collision:: Detected collision!")
                break
        self.logger.info(f'Collision:: TEST PASSED')

    def test_low_priority(self):
        '''test_low_priority

        - publishes 3 msgs on low priority topic
        - makes sure we recv all of them
        '''
        self.logger.info(f'Testing Low Priority')
        # modem1 has some comms_slots that are not aloha slots so we can send lower priority traffic modem1 -> modem0
        queue_publisher = rospy.Publisher('/modem1/test_msg', UInt8, queue_size=5)
        time.sleep(2)

        return_arr = []
        input_arr = []

        msg = UInt8()

        for val in range(3):
            msg.data = val
            queue_publisher.publish(msg) 

            try:
                self.logger.info(f'Low Priority:: Waiting for msg on /modem0/from_acomms/test_msg, timeout=100')
                ret = rospy.wait_for_message(
                    "/modem0/from_acomms/test_msg", UInt8, timeout=100)
            except Exception as e:
                assert str(type(e)) != 'rospy.exceptions.ROSException', f'Unexpected exception: {e}'
            else:
                return_arr.append(ret.data)
                input_arr.append(msg.data)
        
        assert return_arr == input_arr, f"return_arr: {return_arr} != input_arr: {input_arr}"     
        self.logger.info(f'Low Priority:: TEST PASSED')

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """
        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
