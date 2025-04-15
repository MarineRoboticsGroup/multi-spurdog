#!/usr/bin/env python3

import rospy
from ros_acomms_msgs.msg import TdmaScriptedStatus
from tdma_advanced_node import TdmaAdvancedMacNode
from std_msgs.msg import String
import json
import time

from ros_acomms.cfg import tdma_scriptedConfig
from ros_acomms.cfg import tdma_advancedConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

from mac_utils import parse_rospy_yaml_config, FDPMaxBytes4Rate, PSK_ROS_FDP_Rates # pure python class for non-ros functionality

class TdmaScriptedMacNode(TdmaAdvancedMacNode):
    """
    An advanced TDMA MAC node that can execute a scripted test plan.

    Inherits from TdmaAdvancedMacNode.

    Attributes:
    - scripted_test_plan (dict): The scripted test plan.
    - tx_total (int): The total number of transmissions to be made (when no script is passed)
    - packet_length_seconds (float): The duration in seconds of a packet.

    Methods:
    - __init__(self, subclass=False): Initializes the TdmaScriptedMacNode instance.
    - get_test_params(self): Retrieves the test parameters based on the current test index.
    - set_rate_max_bytes_for_next_tx(self): Sets the rate and maximum bytes for the next transmission according to the test parameters.
    - handle_packet_duration(self, packet_duration_seconds): Handles updating the packet duration before calculating the remaining time (if a tdma_scripted_test_plan is passed, otherwise uses the rates passed to the node).
    - generate_tdma_scripted_status(self, basic_msg, scripted_msg): Generates a TDMA scripted status message.
    - spin(self): Main loop for the TDMA scripted MAC node.
    """
    def __init__(self, subclass=False):
        if not subclass:
            rospy.init_node('tdma_scripted_mac') #, log_level=rospy.DEBUG)
            self.tdma_status_publisher = rospy.Publisher('~tdma_scripted_status' if rospy.get_param('~publish_private_status', False) else 'tdma_scripted_status', TdmaScriptedStatus, queue_size=0)
            self.tdma_modem_cmd_publisher = rospy.Publisher('nmea_to_modem', String, queue_size=10, latch=True)
        super().__init__(subclass=True)

        # do scripted stuff
        # scripted_test_plan is deprecated. Use scripted_test_plan_file now
        scripted_test_plan = rospy.get_param('~tdma_scripted_test_plan', None)
        scripted_test_plan_file = rospy.get_param('~tdma_scripted_test_plan_file', None) 

        if scripted_test_plan is not None and scripted_test_plan_file is not None:
            # user passed deprecated param and new param. Raise error
            err = f'FATAL: User passed deprecated param: \"tdma_scripted_test_plan\" AND new param \"tdma_scripted_test_plan_file\". Remove \"tdma_scripted_test_plan\" param.'
            rospy.logfatal(err)
            raise RuntimeWarning(err)
        elif scripted_test_plan is not None:
            # user only passed deprecated param
            rospy.logwarn(f'DEPRECATION WARNING: loading the ros_param: tdma_scripted_test_plan is deprecated.')
            rospy.logwarn(f'DEPRECATION WARNING: use rosparam: tdma_scripted_test_plan_file (str) yaml filepath')
            rospy.logwarn(f'DEPRECATION WARNING: using tdma_scripted_test_plan_file also allows for:')
            rospy.logwarn(f'DEPRECATION WARNING:    - parsing with rospy-yaml-include')
            rospy.logwarn(f'DEPRECATION WARNING:    - re-parsing after launch using dynamic_reconfigure')
        elif scripted_test_plan_file:
            # user passed file path. use rospy yaml to parse test_plan
            scripted_test_plan = parse_rospy_yaml_config(scripted_test_plan_file)

        # by default TDMA scripted only cycles the test plan and considers a tx_count +1 when we have sent a packet
        # ..if being used to send modem commands and no packets, this allows test plan entries to send modem commands instead of packets
        self.cycle_on_tx_packet = rospy.get_param('~cycle_on_tx_packet', True)

        if scripted_test_plan is not None:
            # we have a test_plan dict, make sure it's valid
            self.scripted_test_plan = self.validate_test_plan(scripted_test_plan=scripted_test_plan, raise_exc_on_invalid=True)
        else:
            self.scripted_test_plan = scripted_test_plan

        self.scripted_test_plan_file = scripted_test_plan_file
        self.scripted_test_plan_enabled = rospy.get_param('~tdma_scripted_test_plan_enabled', bool(self.scripted_test_plan))
        self.this_cycle_modem_commands, self.last_cycle_modem_commands = None, None

        log_lines = [
            f'scripted_test_plan_enabled:. {self.scripted_test_plan_enabled}',
            f'scripted_test_plan:......... {json.dumps(self.scripted_test_plan, indent=2, default=str)}',
        ]
        if self.scripted_test_plan_enabled:
            [rospy.loginfo(f"INFO: {ll}") for ll in log_lines]
        else:
            [rospy.logdebug(f"DEBUG: {ll}") for ll in log_lines]

        #   Always burn first active slot. For some reason the very first active slot,
        #   .. can sometimes be a partial slot, i.e., our first active slot is partially over by the time the node is ready to TX
        #   .. this assures we get cycling rates in deterministic way and user can expect rates to cycle in order of test plan
        # 
        self.burned_first_active_slot = False
        self.tx_total = -1 # for case when no scripted plan is passed
        if self.scripted_test_plan:
            # we have a scripted test plan! set cycle counter starting index to 0
            self.current_test_index = 0
            self.tx_count = 0
            self.tx_total = 1
        else:
            # when we aren't using a test plan or it isn't active at the start, we don't need to burn this slot
            self.burned_first_active_slot = True

        if not subclass:
            self.first_dynamic_reconf = True
            self.first_dynamic_reconf_scripted = True
            self.reconfigure_super_class_server = DynamicReconfigureServer(type=tdma_advancedConfig, callback=self.reconfigure, namespace='advanced')
            self.reconfigure_server = DynamicReconfigureServer(type=tdma_scriptedConfig, callback=self.reconfigure_scripted, namespace='scripted')

            try:
                rospy.loginfo(f'INFO: active_slots: {self._active_slots_range}')
                rospy.loginfo(f'INFO: comms_slots: {self._comms_slots_range}')
                rospy.loginfo(f'INFO: nav_slots: {self._nav_slots_range}')
                
                rospy.logdebug(f'DEBUG: nav_slots expanded:\n{self.nav_slots}')
                rospy.logdebug(f'DEBUG: active_slots expanded:\n{self.active_slots}')
                rospy.logdebug(f'DEBUG: comms_slots expanded:\n{self.comms_slots}') 
            except:
                pass

            rospy.loginfo("~*+.$ tdma_scripted_mac node running.")
            self.spin()

    def validate_test_plan(self, scripted_test_plan, raise_exc_on_invalid=False):
        try:
            for test_params in scripted_test_plan['test_cycle']:
                # if the rate (mini/data) are not 1,3,5 they MUST specify maxbytes
                if 'miniframe_rate' in test_params and test_params['miniframe_rate'] not in [1,3,5]:
                    assert 'maximum_miniframe_bytes' in test_params
                if 'dataframe_rate' in test_params and test_params['dataframe_rate'] not in [1,3,5]:
                    assert 'maximum_dataframe_bytes' in test_params
        except AssertionError:
            err = f'FATAL: Invalid test plan. Any rate not [1,3,5] MUST specify max bytes. test_plan: {scripted_test_plan}'
            rospy.logfatal(err)
            if raise_exc_on_invalid: 
                raise AttributeError(err)
        else:
            return scripted_test_plan

    def reconfigure_scripted(self, config, level):
        if self.first_dynamic_reconf_scripted:
            self.first_dynamic_reconf_scripted = False
            config['scripted_test_plan_enabled'] = self.scripted_test_plan_enabled
            config['scripted_test_plan_file'] = str(self.scripted_test_plan_file)
            config['parse_test_plan_file_again'] = False
            rospy.logdebug(f'DEBUG: First dynamic_reconfigure call, syncing config from init')
            return config

        # check and see if we are going from a disabled to enabled state
        if not self.scripted_test_plan_enabled and config['scripted_test_plan_enabled']:
            # TODO:
            # # only need to set this flag again if the enable happened during an active slot. Otherwise, nbd
            # if self.we_are_active:
            #     self.burned_first_active_slot = False
            #     rospy.logwarn(f'scripted_test_plan_enabled is now true and we_are_active with enough time for a packet. Setting self.burned_first_active_slot = False')
            self.burned_first_active_slot = False
            rospy.logwarn(f'NOTICE: scripted_test_plan_enabled is now true after being false. Burning first active slot after this...')

        self.scripted_test_plan_enabled = config['scripted_test_plan_enabled']
        if config['parse_test_plan_file_again']:
            if config['scripted_test_plan_file'] != str(self.scripted_test_plan_file):
                rospy.loginfo(f'INFO: attempting to parse new tdma test plan yaml: {config["scripted_test_plan_file"]}')
                scripted_test_plan = parse_rospy_yaml_config(config['scripted_test_plan_file'])
                if scripted_test_plan:
                    scripted_test_plan = self.validate_test_plan(scripted_test_plan=scripted_test_plan)
                    if scripted_test_plan is not None:
                        self.scripted_test_plan = scripted_test_plan
                    else:
                        rospy.logwarn(f'WARNING: Error with scripted_test_plan_file. Not updating test plan...')
                        config['scripted_test_plan_file'] = self.scripted_test_plan
                else:
                    rospy.logwarn(f'WARNING: Error with scripted_test_plan_file. Not updating test plan...')
                    config['scripted_test_plan_file'] = self.scripted_test_plan
            elif self.scripted_test_plan_file:
                self.scripted_test_plan_file = parse_rospy_yaml_config(self.scripted_test_plan_file)
                rospy.loginfo(f'INFO: Re-parsing same tdma test plan yaml: {self.scripted_test_plan_file}')
            else:
                rospy.logwarn(f'WARNING: Asked to re-parse test plan yaml but using deprecated param tdma_scripted_test_plan. Use tdma_scripted_test_plan_file to use this feature')

        config['parse_test_plan_file_again'] = False
        return config

    def get_test_params(self):
        if self.scripted_test_plan and self.scripted_test_plan_enabled:
            if self.current_test_index >= len(self.scripted_test_plan['test_cycle']):
                # back to the begining
                self.current_test_index = 0
                self.tx_count = 0

            test_params = self.scripted_test_plan['test_cycle'][self.current_test_index]
            self.tx_total = test_params.get('packet_tx_count', 1)
            self.tx_count += 1

            rospy.logdebug(f'DEBUG: Current test params: {test_params}')
            # keep default behavior rather than keeping prior test setting
            self.cycle_on_tx_packet = test_params.get('cycle_on_tx_packet', True)

            fdp_mini = FDPMaxBytes4Rate(
                    rate=test_params.get('miniframe_rate', self.miniframe_rate), 
                    max_bytes=test_params.get(
                        'maximum_miniframe_bytes', 
                        PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_MINI_RATE[test_params.get('miniframe_rate', self.miniframe_rate)].default
                    ), 
                    default=None,
                    packet_duration_ms=PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_MINI_RATE[test_params.get('miniframe_rate', self.miniframe_rate)].packet_duration_ms
                )
            fdp_data = FDPMaxBytes4Rate(
                    rate=test_params.get('dataframe_rate', self.dataframe_rate), 
                    max_bytes=test_params.get(
                        'maximum_dataframe_bytes', 
                        PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_DATA_RATE[test_params.get('dataframe_rate', self.dataframe_rate)].default
                    ), 
                    default=None,
                    packet_duration_ms=PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_DATA_RATE[test_params.get('dataframe_rate', self.dataframe_rate)].packet_duration_ms
                )
            script_packet_duration_sec = test_params.get('packet_duration_sec', (fdp_mini.packet_duration_ms + fdp_data.packet_duration_ms) // 1000)

            # check and see if there are modem NMEA strings assiociated with this test
            if 'modem_commands' in test_params:
                # optional. If passed, a single string or list of strings is expected
                try:
                    if isinstance(test_params['modem_commands'], str):
                        test_params['modem_commands'] = [test_params['modem_commands']]
                except:
                    self.this_cycle_modem_commands = []
                    self.last_cycle_modem_commands = []
                    rospy.logerr(f'ERROR: issue with modem command in test plan')
                else:
                    self.last_cycle_modem_commands = self.this_cycle_modem_commands
                    self.this_cycle_modem_commands = test_params['modem_commands']
            else:
                self.this_cycle_modem_commands = []

            if self.tx_total == 1:
                tx_count_local = 1
            else:
                tx_count_local = self.tx_count

            if self.tx_count >= self.tx_total or self.tx_total == 1:
                rospy.logdebug(f'DEBUG: Last TX with these settings')
                # last tx with these setting
                self.current_test_index += 1
                self.tx_count = 0 # set state for next call
        else:
            tx_count_local = -1
            fdp_mini = FDPMaxBytes4Rate(rate=self.miniframe_rate, max_bytes=self.maximum_miniframe_bytes, default=None, packet_duration_ms=PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_MINI_RATE[self.miniframe_rate].packet_duration_ms)
            fdp_data = FDPMaxBytes4Rate(rate=self.dataframe_rate, max_bytes=self.maximum_dataframe_bytes, default=None, packet_duration_ms=PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_MINI_RATE[self.dataframe_rate].packet_duration_ms)
            script_packet_duration_sec = (fdp_mini.packet_duration_ms + fdp_data.packet_duration_ms)

        return fdp_mini, fdp_data, tx_count_local, script_packet_duration_sec

    def set_rate_max_bytes_for_next_tx(self):
        fdp_mini, fdp_data, tx_count_local, script_packet_duration_ms = self.get_test_params()

        msg = TdmaScriptedStatus()
        msg.last_miniframe_rate = self.miniframe_rate
        msg.last_dataframe_rate = self.dataframe_rate
        msg.last_maximum_miniframe_bytes = self.maximum_miniframe_bytes
        msg.last_maximum_dataframe_bytes = self.maximum_dataframe_bytes
        msg.packet_duration_sec = self.packet_length_seconds

        msg.this_cycle_tx_count = tx_count_local # num of times we have tx'd with the current settings (including this time)
        msg.this_cycle_tx_total = self.tx_total  # num of times we will use these settings before moving on to the next test

        msg.this_cycle_modem_commands = self.this_cycle_modem_commands if self.this_cycle_modem_commands else []
        msg.last_cycle_modem_commands = self.last_cycle_modem_commands if self.last_cycle_modem_commands else []
        # now publish this cycles modem commands (which is actually last_cycle_modem_commands)
        if self.last_cycle_modem_commands:
            for modem_cmd in self.last_cycle_modem_commands:
                # if self.burned_first_active_slot:
                #     # only publish modem commands if we have already burned out first slot
                self.tdma_modem_cmd_publisher.publish(data=modem_cmd)
                rospy.loginfo(f'INFO: Just sent modem command: {modem_cmd}')

        # now set class attrs for next tx according to test yaml
        self.miniframe_rate = fdp_mini.rate
        self.dataframe_rate = fdp_data.rate
        self.maximum_miniframe_bytes = fdp_mini.max_bytes
        self.maximum_dataframe_bytes = fdp_data.max_bytes

        if tx_count_local != -1:
            rospy.logdebug(f"DEBUG: TX {tx_count_local} /of/ {self.tx_total} times. Returning: {fdp_mini}, {fdp_data}, {tx_count_local}")
            rospy.logdebug(f'DEBUG: Changed rate and max bytes for next TX:\n{msg}')

        return script_packet_duration_ms, msg

    def handle_packet_duration(self, packet_duration_seconds):
        # if we are running a test plan update the packet_length_seconds from the test plan (or default for rate)
        if self.scripted_test_plan_enabled:
            self.packet_length_seconds = packet_duration_seconds

    def generate_tdma_scripted_status(self, basic_msg, scripted_msg):
        advanced_msg = self.generate_tdma_advanced_status(basic_msg=basic_msg)
        for slot in advanced_msg.__slots__:
            setattr(scripted_msg, slot, getattr(advanced_msg, slot))
        return scripted_msg

    def spin(self):
        rate = rospy.Rate(5)
        last_tx_time = 0
        msg = None
        sent_modem_pings = False
        sent_transponder_ping = False

        # Setup prior to first Transmit. 
        #   * Set dataframe_rate, miniframe_rate, packet_duration_sec, etc... 
        #   * If test plan is provided & enabled we'll get the prior values from the first test in the test plan
        #   * If there is no test plan or it's not enabled, we end up using the prior values passed in the launch file (or defaults if none passed)
        packet_duration_sec, scripted_msg = self.set_rate_max_bytes_for_next_tx()

        while not rospy.is_shutdown():
            self.handle_packet_duration(packet_duration_seconds=packet_duration_sec)
            msg = self.get_current_slot_info(software_mute=self.software_mute)
            self.tdma_status_publisher.publish(self.generate_tdma_scripted_status(basic_msg=msg, scripted_msg=scripted_msg))

            if self.can_transmit_now(msg=msg, last_tx_time=last_tx_time):
                # make sure we have burned out first active slot
                if not self.burned_first_active_slot:
                    rospy.logwarn(f'NOTICE: TDMA Scripted burning first active slot to assure test plan order.')
                    rospy.sleep(msg.remaining_active_seconds - 1) 
                    # we'll wake up without time for a TX. only hit this clause again if,
                    # .. we disable then enable to test_plan, we burn the first active slot after the toggle
                    self.burned_first_active_slot = True
                    continue

                # we are active and have at least enough time to send another packet
                sent_modem_pings, sent_transponder_ping, update_last_tx = self.handle_queuing_packet(
                        msg.current_slot, 
                        msg.remaining_active_seconds, 
                        sent_modem_pings, 
                        sent_transponder_ping
                    )

                if not self.cycle_on_tx_packet and not update_last_tx:
                    # this tdma is configured to cycle the test params even when it did not tx a packet
                    # .. we create the down time the packet would take (defined in the test plan) with a sleep call
                    rospy.logwarn(f'NOTICE: TDMA Scripted cycling test plan even though there was no packet. Sleeping for {self.packet_length_seconds} sec...')
                    # !! rospy.sleep will take MUCH longer on systems when very little compute pwr because, ROS is actually holding up 
                    # !! .. rospy.sleep(self.packet_length_seconds) when it's waiting on CPU cycles.. Instead run time.sleep.
                    # !! .. this means this feature should not be used in sim when running faster than real time.
                    # rospy.sleep(self.packet_length_seconds)
                    time.sleep(self.packet_length_seconds)
                    update_last_tx = True

                if update_last_tx:
                    packet_duration_sec, scripted_msg = self.set_rate_max_bytes_for_next_tx()
                    last_tx_time = rospy.get_time()
                else:
                    # nothing sent, sleep then check again
                    rospy.logdebug_throttle(10, f'DEBUG: Nothing was sent even though we are active. We may have nothing to send or traffic that we have to send is not allowed in this slot.')
                    rate.sleep()
                    continue
            else:
                # not enough time. sleep for rate and try again
                rate.sleep()
                continue


if __name__ == "__main__":
    try:
        tdma_state_node = TdmaScriptedMacNode()
        rospy.loginfo("tdma_scripted_node shutdown (interrupt)")
    except rospy.ROSInterruptException:
        rospy.loginfo("tdma_scripted_node shutdown (interrupt)")


