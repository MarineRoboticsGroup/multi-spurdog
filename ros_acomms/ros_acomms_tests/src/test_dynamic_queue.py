#! /usr/bin/env python3


'''
test_dynamic_queue
------------------

tests for dynamic queue in ros accoms
'''


import os
import sys
import pytest
import rospy
import rospkg
import roslaunch
from std_msgs.msg import UInt8

from ros_acomms_msgs.srv import GetNextQueuedMessage, GetNextQueuedMessageRequest, GetNextQueuedMessageResponse
from ros_acomms_msgs.msg import UpdateQueuedMessage

class Helper():
    def __init__(self) -> None:
        rospy.Service('modem0/get_next', GetNextQueuedMessage, self.handle_get_next)
        rospy.Subscriber('modem0/update_queue', UpdateQueuedMessage, self.on_update_queue)  

        rospy.Service('modem1/get_next', GetNextQueuedMessage, self.handle_get_next)
        rospy.Subscriber('modem1/update_queue', UpdateQueuedMessage, self.on_update_queue)
        self.hasData = None

    def handle_get_next(self, req: GetNextQueuedMessageRequest) -> None :
        '''
        :param req: service request data 
        ''' 
        if self.hasData is not None:
            payload = bytes([6])
            self.hasData = None if self.hasData == 0 else self.hasData-1
            return GetNextQueuedMessageResponse(has_message=True,
                                                    message_id=0,
                                                    dest_address=121,
                                                    priority=10,
                                                    data=payload,
                                                    data_size_in_bits=len(payload)*8,
                                                    )
            
        else:
            return GetNextQueuedMessageResponse(has_message=False)
    
    def on_update_queue(self, update_msg: UpdateQueuedMessage):
        pass


class TestDynamicQueue:
    '''ros_acomms dynamic queue test class'''
    
    @classmethod
    def setup_class(self):
        """
        setup function for tests
        """

        rospy.init_node("test_dynamic_queue", log_level=rospy.INFO)

        tdma_type = rospy.get_param("/tdma_type", "tdma")

        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/dynamic_queue_test.launch",
        )
        self.helper = Helper()

        roslaunch_args = [f"tdma_type:={tdma_type}"]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start() 
    

    def test_message_recieved(self):
        '''
        Test for simple dynamic queue message, test for message transmission and marked as transmitted
        '''
        self.helper.hasData = 2
        updateMsg = rospy.wait_for_message(
                    "/modem0/update_queue", UInt8, timeout=100)
        ret = rospy.wait_for_message(
                   "/modem1/from_acomms/test_dynamic_msg", UInt8, timeout=100)
        assert ret.data == 6, f"return: {ret.data}" 
        assert updateMsg.event == 2, f"update msg event not marked as transmitted"

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """

        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
