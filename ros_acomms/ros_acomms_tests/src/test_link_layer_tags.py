#! /usr/bin/env python3
'''
test_link_layer_tags
------------------

tests for dynamic queue in ros accoms
'''
import os
import time
import sys
import pytest
import rospy
import roslib.message
import rospkg
import roslaunch
from std_msgs.msg import UInt8, Int8
import logging
from rospy.msg import AnyMsg
import traceback

from ros_acomms_msgs.srv import GetNextQueuedMessage, GetNextQueuedMessageRequest, GetNextQueuedMessageResponse
from ros_acomms_msgs.msg import UpdateQueuedMessage, QueueStatus

# sys.path.insert(0, '../../ros_acomms/src/')
# from mac_utils import deserialze_anymsg
def deserialze_anymsg(msg_data: AnyMsg):
    topic_type = msg_data._connection_header['type']
    topic_class = roslib.message.get_message_class(topic_type)
    msg = topic_class()
    msg.deserialize(msg_data._buff)
    return msg

class Helper():
    def __init__(self, logger, tags=None, post_fix='') -> None:
        rospy.Service(f'modem0/get_next{post_fix}', GetNextQueuedMessage, self.handle_get_next)
        rospy.Subscriber(f'modem0/update_queue{post_fix}', UpdateQueuedMessage, self.on_update_queue)

        rospy.Service(f'modem1/get_next{post_fix}', GetNextQueuedMessage, self.handle_get_next)
        rospy.Subscriber(f'modem1/update_queue{post_fix}', UpdateQueuedMessage, self.on_update_queue)
        self.has_data = False
        self.tags = tags if tags else []
        self.logger = logger
        self.post_fix = post_fix
        self.pass_empty_msg_tags = False

    def handle_get_next(self, req: GetNextQueuedMessageRequest) -> None :
        '''
        :param req: service request data 
        ''' 
        # self.logger.info(f'dynamic_queues::helper{self.post_fix}::handle_get_next()\nreq: {req}')
        # default if we have no data
        resp = GetNextQueuedMessageResponse(has_message=False)

        if not self.has_data:
            self.logger.warning(f'dynamic_queues::helper{self.post_fix}::no data to return')

        elif set(req.exclude_tags).intersection(set(self.tags)):
            # if any of this queues tags are in the exclude list
            self.logger.warning(f'dynamic_queues::helper{self.post_fix}::we have tags in excluded list! tags: {self.tags}')

        elif req.require_tags != [] and not set(req.require_tags).intersection(set(self.tags)):
            # if require tags is not an empty list and we do not have a tag in the list
            self.logger.warning(f'dynamic_queues::helper{self.post_fix}::we do NOT have tags in required list! tags: {self.tags}')

        else:
            # we're clear to return data this time
            # self.logger.warning(f'dynamic_queues::helper{self.post_fix}::we are returning data! tags: {self.tags}')
            payload = bytes([6])
            resp = GetNextQueuedMessageResponse(has_message=True,
                                                message_id=0,
                                                dest_address=121,
                                                priority=100,
                                                data=payload,
                                                data_size_in_bits=len(payload)*8,
                                                msg_tags=[] if self.pass_empty_msg_tags else self.tags,
                                                )
            
        return resp
    
    def on_update_queue(self, update_msg: UpdateQueuedMessage):
        pass

class TestLinkLayerTags:
    '''ros_acomms link layer tags test class'''
    
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """

        rospy.init_node("test_link_layer_tags", log_level=rospy.INFO)
        self.logger = logging.getLogger("rosout")

        # self.tdma_type = rospy.get_param("/tdma_type", "tdma_slotted_aloha")
        self.tdma_type = rospy.get_param("/tdma_type", "tdma_advanced")

        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/test_link_layer_tags.launch",
        )
        self.helper = Helper(logger=self.logger, tags=['extra_special', 'chat', 'some_aloha'])
        self.helper_exclude = Helper(logger=self.logger, tags=['always_exclude'], post_fix='_exclude')

        self.modem_last = None

        roslaunch_args = [f"tdma_type:={self.tdma_type}"]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()

        time.sleep(5)
        self.logger.warning('Node: test_link_layer_tags running!')

    def _wait_until_slots(self, modem_ns, slots, slots_are_exclusive=False):
        tdma_status_sub = rospy.Subscriber(f'/{modem_ns}/{self.tdma_type}_status', AnyMsg, callback=self._topic_callback)
        try:
            while not rospy.is_shutdown():
                if self.modem_last is None:
                    rospy.sleep(1.0)
                    continue

                current_slot = self.modem_last.current_slot

                if slots_are_exclusive:
                    if current_slot not in slots:
                        self.logger.warning(f"DONE: Slots are exclusive and our current_slot: {current_slot} is not in slots: {slots}")
                        break 
                else:
                    if current_slot in slots:
                        self.logger.warning(f"DONE: We are in range, slots: {slots}, current_slot: {current_slot}")
                        break
                
                rospy.sleep(self.modem_last.remaining_slot_seconds)
        except:
            self.logger.warning(f"Exception thrown in _wait_until_slots() {traceback.format_exc()}")
        else:
            current_slot = self.modem_last.current_slot
            remaining_slot_seconds = self.modem_last.remaining_slot_seconds
            return remaining_slot_seconds, current_slot

        finally:
            self.modem_last = None
            tdma_status_sub.unregister()

    def _topic_callback(self, msg):
        topic = msg._connection_header['topic']
        self.modem_last = deserialze_anymsg(msg)

    def _get_queue_summary(self, queue_status, target_queue_id):
        for queue_summary in queue_status.queue_summaries:
            if queue_summary.queue_id == target_queue_id:
                self.logger.warning(f"Got queue_summary for id: {target_queue_id}: {queue_summary}")
                return queue_summary

    def test_message_received(self):
        '''
        Test for simple dynamic queue message, test for message transmission and marked as transmitted
        '''
        self.logger.warning(f'Testing dynamic queue message with allowed tags')
        self.helper.has_data = True

        try:
            updateMsg = rospy.wait_for_message(
                    "/modem0/update_queue", UInt8, timeout=100)
            ret = rospy.wait_for_message(
                    "/modem1/from_acomms/test_dynamic_msg", UInt8, timeout=100)
        except rospy.exceptions.ROSException:
            err = f'Testing dynamic queue message with allowed tags FAILED! Timed-out waiting for message...'
            self.logger.info(err)
            pytest.fail(err)
        else:
            assert ret.data == 6, f"return: {ret.data}" 
            assert updateMsg.event == 2, f"update msg event not marked as transmitted"
            self.logger.info(f'Testing dynamic queue message with allowed tags PASSED!')
        finally:
            self.logger.warning(f'Testing dynamic queue message with allowed tags!')
            self.helper.has_data = False

    def test_message_not_received(self):
        '''
        Test for simple dynamic queue message, test for message transmission and marked as transmitted
        '''
        self.logger.warning(f'Testing dynamic queue message with excluded tags')
        self.helper_exclude.has_data = True

        try:
            ret = rospy.wait_for_message(
                       "/modem1/from_acomms/test_dynamic_msg_exclude", UInt8, timeout=50) # this waits for 5 whole slots...
        except rospy.exceptions.ROSException:
            self.logger.info(f'Testing dynamic queue message with excluded tag: always_exclude, PASSED!')
        else:
            err = f'Testing dynamic queue message with excluded tag: always_exclude, FAILED! we got a message'
            self.logger.info(err)
            pytest.fail(err)
        finally:
            self.logger.warning(f'Testing dynamic queue message with excluded tags!')
            self.helper_exclude.has_data = False

    def test_message_invalid_msg_tags(self):
        '''
        Test for simple dynamic queue message, test for message transmission and marked as transmitted
        '''
        self.logger.warning(f'Testing dynamic queue message with allowed tags AND pass_empty_msg_tags=True')
        self.logger.warning(f'This test succeeds when we CANNOT get a message queued because our response does not include msg_tags!')
        self.logger.warning(f'This test is for checking that message_queue is validating the GetNextQueuedMessageResponse.msg_tags,')
        self.logger.warning(f'.. with the tags passed by the MAC. Making sure message_queue handles the case where a dynamic_queue client ignores QoS tags')
        self.helper.pass_empty_msg_tags = True
        self.helper.has_data = True

        try:
            updateMsg = rospy.wait_for_message(
                    "/modem0/update_queue", UInt8, timeout=50)
        except rospy.exceptions.ROSException:
            self.logger.info(f'Testing dynamic queue message with allowed tags AND pass_empty_msg_tags=True SUCCEEDED!')
        else:
            err = f'Testing dynamic queue message with allowed tags AND pass_empty_msg_tags=True FAILED!'
            self.logger.info(err)
            pytest.fail(err)
        finally:
            self.logger.warning(f'Testing dynamic queue message with allowed tags AND pass_empty_msg_tags=True')
            self.helper.pass_empty_msg_tags = False
            self.helper.has_data = False

    def test_link_layer_queue_exclude(self):
        self.logger.warning(f'Testing static queue message with \"always_exclude\" tag')

        publisher = rospy.Publisher('/modem0/low_priority', Int8, queue_size=15, latch=True)
        time.sleep(2)
        msg = Int8(data=120)
        [publisher.publish(msg) for _ in range(15)] # this tests that our queue_maxsize cuts off correctly too!
        rospy.sleep(20)

        try:
            ret = rospy.wait_for_message(f'/modem0/queue_status', QueueStatus, timeout=50)
            # get first queue_summary for low_priority after publishing 15 msgs (maxlen 10)
            queue_summary = self._get_queue_summary(queue_status=ret, target_queue_id=101)
            # queue should have 10 messages queued
            assert queue_summary.message_count == 10, f'FAILURE! /modem0/low_priority should have 10 messages queued! {queue_summary}'
            self.logger.warning("Partial success, we have 10 messages queued in /modem0/low_priority...")
            self.logger.warning("Sleeping for another 30 sec (will be 50 total, ~half cycle wait like dynamic Q test)")
            self.logger.warning("Then, checking that there are still 10 messages queued in /modem0/low_priority")
            rospy.sleep(30)

            # now get the latest queue_status and make sure we still have 10 messages in the queue
            ret = rospy.wait_for_message(f'/modem0/queue_status', QueueStatus, timeout=50)
            queue_summary = self._get_queue_summary(queue_status=ret, target_queue_id=101)
            assert queue_summary.message_count == 10, f'FAILURE! /modem0/low_priority should have 10 messages queued! {queue_summary}'
            self.logger.warning("SUCCEESS! We still have 10 messages queued in /modem0/low_priority after a full cycle!")

        except rospy.exceptions.ROSException:
            err = "FAILURE: timed out while waiting for /modem0/queue_status... this should not happen..."
            self.logger.warning(err)
            pytest.fail(err)

    def test_link_layer_queue_require(self):
        self.logger.warning(f'Testing static queue message with \"extra_special\" tag')

        # only do this if we are running slotted aloha, otherwise tdma only uses tags for filtering and sets active slots to 0:11
        if self.tdma_type == 'tdma_slotted_aloha':
            self.logger.warning(f'Waiting until we are in one of these slots: 0,4-11:1 for modem0 so we can set a tight timeout for high_priority_chat_msgs...')
            remaining_slot_seconds, current_slot = self._wait_until_slots(modem_ns='modem0', slots=[1,2,3], slots_are_exclusive=True)
            if remaining_slot_seconds < 8:
                remaining_slot_seconds = 8
        else:
            remaining_slot_seconds = 10

        publisher = rospy.Publisher('/modem0/high_priority_chat_msgs', Int8, queue_size=20, latch=True)
        time.sleep(2)
        start_val = 100
        for _ in range(20):
            publisher.publish(data=start_val)
            start_val += 1

        self.logger.warning(f'Just published 20 messages on /modem0/high_priority_chat_msgs. They should go out next slot')
        if self.tdma_type == 'tdma_slotted_aloha':
            self.logger.warning(
                f'Timeout for /modem1/from_acomms/high_priority_chat_msgs is tight. We wait the remaining_slot_seconds:{remaining_slot_seconds:.2f} plus 1 slot (10 sec)')
        try:
            start_t = time.time()
            ret = rospy.wait_for_message(f'/modem1/from_acomms/high_priority_chat_msgs', Int8, timeout=remaining_slot_seconds + 10)
            stop_t = time.time()
            self.logger.warning(f"INFO: how long rospy.wait_for_message(/modem1/from_acomms/high_priority_chat_msgs) took: {stop_t - start_t:.2f} our timeout was: {remaining_slot_seconds + 10:.2f}") 
        except rospy.exceptions.ROSException:
            err = "Timed-out waiting for msg on /modem1/from_acomms/high_priority_chat_msgs..."
            self.logger.warning(err)
            pytest.fail(err)
        else:
            assert ret.data == 100, f'FAILURE, msg on /modem1/from_acomms/high_priority_chat_msgs is not first value sent (100), ret: {ret.data}'
            self.logger.warning(f"Success! We got a chat_msg from modem0 on: /modem1/from_acomms/high_priority_chat_msgs == 100 ret: {ret.data}")            

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """

        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
