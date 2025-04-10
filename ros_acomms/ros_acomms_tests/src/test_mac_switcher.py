#! /usr/bin/env python3

'''
tests for mac_switcher
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

from std_msgs.msg import Header, Bool
from ros_acomms_msgs.msg import TdmaAdvancedStatus, TdmaScriptedStatus, TdmaSlottedAlohaStatus, MacSwitcherStatus

class TestMacSwitcher:
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
        rospy.init_node("test_mac_switcher", log_level=rospy.DEBUG)
        self.logger = logging.getLogger("rosout")

        self.tdma_type = rospy.get_param("/tdma_type", "tdma_advanced")
        if self.tdma_type not in TestMacSwitcher.VALID_TDMA_TYPES:
            raise NotImplementedError(f'This test module does not work with {self.tdma_type}. Valid tdma_types: {TestMacSwitcher.VALID_TDMA_TYPES}')

        self.roslaunch_dict = dict(
                num_slots=2,
                slot_duration_seconds=6,
                tdma_type=self.tdma_type,
            )
        self.rospack = rospkg.RosPack()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file = os.path.join(
            self.rospack.get_path("ros_acomms_tests"),
            "launch/test_mac_switcher.launch",
        )

        roslaunch_args = [f"{k}:={v}" for k, v in self.roslaunch_dict.items()]
        full_cmd_list = [launch_file] + roslaunch_args
        roslaunch_file_param = [(roslaunch.rlutil.resolve_launch_arguments(full_cmd_list)[0], roslaunch_args)]

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file_param)
        self.launch.start()

        self.one_cycle_duration = self.roslaunch_dict['slot_duration_seconds'] * self.roslaunch_dict['num_slots']
        self.logger.warning(f'test_mac_switcher one_cycle_duration ({self.one_cycle_duration} sec). Sleeping for 2.0')
        rospy.sleep(2.0)
        self.logger.warning('Node: test_mac_switcher running!')

    def test_all_systems_nominal(self):
        self.logger.warning(f'Testing Multi Mac simple launch check, and both tdma managers are up (and all managed tdmas are up)')

        ret0, ret1 = None, None
        try:
            ret0 = rospy.wait_for_message(f'/modem0/mac_switcher_status', MacSwitcherStatus, timeout=100)
            ret1 = rospy.wait_for_message(f'/modem1/mac_switcher_status', MacSwitcherStatus, timeout=100)
        except rospy.exceptions.ROSException:
            self.logger.warning("timed out while waiting for mac_switcher_status")
            self.logger.warning(f"        modem0: {ret0}")
            self.logger.warning(f"        modem1: {ret1}")
            pytest.fail("timed out while waiting for mac_switcher_status")
        else:
            self.logger.warning("Success both nodes are up!")
            self.logger.warning(f"        modem0: {ret0}")
            self.logger.warning(f"        modem1: {ret1}")


    # def test_start_state(self):
        # self.logger.warning(f'Testing Multi Mac on modem0')
        # self.logger.info(f'Testing Multi Mac:: Sleep for 2.0, then test all managed tdma nodes are software_mute:True and default_mac_namespace is software_mute:False')
        # rospy.sleep(2.0)
        
        # # tdma_A_select_publisher = rospy.Publisher('/modem0/tdma_A/select', Bool, queue_size=5)
        # # rospy.sleep(2.0)
        # # tdma_A_select_publisher.publish(data=True)
        # # rospy.sleep(1.0)

        # while not rospy.is_shutdown():
        #     try:
        #         self.logger.info(f'Testing Multi Mac:: Waiting for next message on /modem0/mac_switcher_status. timeout={self.roslaunch_dict["slot_duration_seconds"]}')
        #         ret = rospy.wait_for_message(
        #             f'/modem0/mac_switcher_status', MacSwitcherStatus, timeout=self.roslaunch_dict['slot_duration_seconds'])
                
        #         for managed_tdma in ret.managed_mac_namespaces:
        #             assert managed_tdma.ns in ['tdma', 'tdma_A', 'tdma_B', 'tdma_C'], f'Error: unexpected tdma namespace'
        #             assert managed_tdma.software_mute if ret.default_mac_namespace == managed_tdma.ns, f'Error: default_mac_namespace should be UN-MUTED: {managed_tdma}'
        #             assert not managed_tdma.software_mute if ret.default_mac_namespace != managed_tdma.ns, f'Error: non default_mac_namespace is UN-MUTED. All managed tdma namespaces should be MUTED unless they are the default_mac_namespace: {managed_tdma}'
        #         # if we got to here, we pass
        #         self.logger.warning("Testing Multi Mac:: PASSED! all managed tdma nodes are software_mute:True and default_mac_namespace is software_mute:False")
        #         return

        #     except rospy.exceptions.ROSException:
        #         self.logger.warning("timed out while waiting for mac_switcher_status")
        #         pytest.fail("timed out while waiting for mac_switcher_status")

    @classmethod
    def teardown_class(self):
        """
        teardown function for tests
        """
        self.launch.shutdown()

if __name__ == "__main__":
    sys.exit(pytest.main(['--capture=no', __file__]))
