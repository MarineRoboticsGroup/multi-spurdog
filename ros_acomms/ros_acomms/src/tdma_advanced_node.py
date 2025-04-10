#!/usr/bin/env python3

import rospy
from ros_acomms_msgs.srv import PingModem, PingModemRequest, PingModemResponse, PingTransponders, PingTranspondersRequest
from ros_acomms_msgs.msg import TdmaAdvancedStatus
from tdma_node import TdmaMacNode
from std_msgs.msg import Bool, UInt8

from ros_acomms.cfg import tdma_advancedConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

import datetime
import traceback

from mac_utils import PSK_ROS_FDP_Rates, expanded_slots_as_ranges # pure python class for non-ros functionality

class TdmaAdvancedMacNode(TdmaMacNode):
    """
    TDMA MAC node that support dynamic reconfiguration of parameters, modem and transponder pings, and software mute

    Extends TdmaMacNode

    Attributes:
        nav_slots (list): A list of slot for navigation pings.
        comms_slots (list): A list of slot numbers for comms packets.
        software_mute (bool): A flag indicating whether the node is in software mute mode.
        cycle_start_time_offset_sec (float): The offset in seconds for the start time of the TDMA cycle.
        pings_per_slot (int): The number of pings per slot.
        ping_modems (bool): A flag indicating whether to ping modems.
        ping_modem_src (int): The source ID of the modem to ping.
        ping_modem_timeout_sec (int): The timeout in seconds for modem pings.
        ping_cdr (int): The Compact Data Request (see the uModem manual) parameter sent with ping requests.
        ping_transponders (bool): A flag indicating whether to ping transponders.
        ping_transponder_a (bool): A flag indicating whether to ping transponder A.
        ping_transponder_b (bool): A flag indicating whether to ping transponder B.
        ping_transponder_c (bool): A flag indicating whether to ping transponder C.
        ping_transponder_d (bool): A flag indicating whether to ping transponder D.
        transponder_reply_timeout_sec (int): The timeout in seconds for transponder replies.

    Methods:
        wait_for_ping_services(timeout=120)
            Waits for the ping_modem and ping_transponders services to come online.

        config_slot_mask(slots)
            Calculates the slot mask based on the slot type specified.

        reconfigure(config, level)
            Updates the configuration based on dynamic reconfigure requests.
    """
    def __init__(self, subclass=False):
        if not subclass:
            rospy.init_node('tdma_advanced_mac') #, log_level=rospy.DEBUG)
            self.tdma_status_publisher = rospy.Publisher('~tdma_advanced_status' if rospy.get_param('~publish_private_status', False) else 'tdma_advanced_status', TdmaAdvancedStatus, queue_size=0)
        super().__init__(subclass=True)
        # take supers active_slots list of ints and generate comms_slot & nav_slots masks
        self.nav_slots = self.config_slot_mask(slots=rospy.get_param('~nav_slots', None), allow_empty=True)
        self.comms_slots = self.config_slot_mask(slots=rospy.get_param('~comms_slots', None), allow_empty=True)
        self.cache_slot_ranges_for_status()
        
        # do advanced stuff
        self.software_mute = rospy.get_param('~software_mute', None)
        if self.software_mute is None:
            # if unset, set to False and set param server
            self.software_mute = False
            rospy.set_param('~software_mute', False)

        self.cycle_start_time_offset_sec = rospy.get_param('~skew_cycle_sec', 0)
        self.pings_per_slot = rospy.get_param('~pings_per_slot', 0)
        self.ping_modems = rospy.get_param('~ping_modem', bool(self.pings_per_slot))
        self.ping_modem_src = rospy.get_param('~ping_modem_src', 1)
        self.ping_modem_timeout_sec = rospy.get_param('~ping_modem_timeout_sec', 5)
        self.ping_cdr = rospy.get_param('~ping_cdr', 4)

        self.ping_transponders = rospy.get_param('~ping_transponders', False)
        self.ping_transponder_a = rospy.get_param('~ping_transponder_a', True)
        self.ping_transponder_b = rospy.get_param('~ping_transponder_b', True)
        self.ping_transponder_c = rospy.get_param('~ping_transponder_c', True)
        self.ping_transponder_d = rospy.get_param('~ping_transponder_d', True)
        self.transponder_reply_timeout_sec = rospy.get_param('~transponder_reply_timeout_sec', 5)

        self.wait_for_ping_services(timeout=rospy.get_param('~wait_for_services_timeout_sec', None))
        # setup our own ping_modem endpoint for sending pings in tdma
        self.queued_ping_request = None
        self.queued_ping_response = None
        rospy.Service("~ping_modem", PingModem, self.handle_ping_modem)

        # set up subscriber to set software_mute without overhead of dynamic_reconfigure
        rospy.Subscriber('~set_software_mute', Bool, self.on_set_software_mute)
        # set up subscriber for setting auto ping modem src id (can also be set with dynamic reconfigure)
        rospy.Subscriber('~change_auto_ping_modem_src', UInt8, self.on_change_auto_ping_modem_src)

        # this method is called everytime the slot changes. We reset modem ping flags here
        self.sent_modem_pings = False
        self.sent_transponder_ping = False
        self.register_on_slot_change_callback(self.on_slot_changed)

        if not subclass:
            # setup dynamic reconf for periodic ping params
            self.first_dynamic_reconf = True
            self.reconfigure_server = DynamicReconfigureServer(tdma_advancedConfig, self.reconfigure)

            try:
                rospy.loginfo(f'INFO: active_slots: {self._active_slots_range}')
                rospy.loginfo(f'INFO: comms_slots: {self._comms_slots_range}')
                rospy.loginfo(f'INFO: nav_slots: {self._nav_slots_range}')
                
                rospy.logdebug(f'DEBUG: nav_slots expanded:\n{self.nav_slots}')
                rospy.logdebug(f'DEBUG: active_slots expanded:\n{self.active_slots}')
                rospy.logdebug(f'DEBUG: comms_slots expanded:\n{self.comms_slots}') 
            except:
                pass

            rospy.loginfo("~*+^ tdma_advanced_mac node running.")
            self.spin()

    def invalidate_dynamic_reconfigure_cache(self):
        # hacky way of invalidating dynamic reconfigure cache. 
        # this will confuse a user of rqt by seemingly ignoring the next config it passes
        # .. but, it will be updated with the values from the class
        self.first_dynamic_reconf = True

    def on_set_software_mute(self, msg):
        if self.software_mute != bool(msg.data):
            self.invalidate_dynamic_reconfigure_cache()
        self.software_mute = bool(msg.data)
        rospy.set_param('~software_mute', self.software_mute)
        rospy.logwarn(f'NOTICE: on_set_software_mute() software_mute: {self.software_mute}')

    def on_change_auto_ping_modem_src(self, msg):
        if self.ping_modem_src != int(msg.data):
            self.invalidate_dynamic_reconfigure_cache()
        self.ping_modem_src = int(msg.data)
        rospy.set_param('~ping_modem_src', self.ping_modem_src)
        rospy.logwarn(f'NOTICE: on_change_auto_ping_modem_src() ping_modem_src: {self.ping_modem_src}')

    def cache_slot_ranges_for_status(self):
        self._active_slots_range = expanded_slots_as_ranges(slots=self.active_slots)
        self._comms_slots_range = expanded_slots_as_ranges(slots=self.comms_slots)
        self._nav_slots_range = expanded_slots_as_ranges(slots=self.nav_slots)

    def get_formatted_slot_ranges(self):
        return ', '.join([
            f'active_slots: {self._active_slots_range}',
            f'comms_slots: {self._comms_slots_range}',
            f'nav_slots: {self._nav_slots_range}',
        ])

    def wait_for_ping_services(self, timeout=120):
        # wait for ping_modem service to come online
        rospy.loginfo("INFO: tdma_advanced_mac waiting for ping_modem service")
        rospy.wait_for_service('ping_modem', timeout=timeout)
        self.send_modem_ping = rospy.ServiceProxy('ping_modem', PingModem)

        try:
            # since the ping_modem service is available in sim and hw, it blocking waits,
            # as of jan 2024, modem_sim does not have a ping_transponders service.
            # if the user has ping_transponders = True and this times out, it will kill the node
            # if the user has ping_transponders = False, we just print an error if this service times out (which it will for sim)
            rospy.wait_for_service('ping_transponders', timeout=1)
        except rospy.exceptions.ROSException:
            if self.ping_transponders:
                raise # raise error right away, otherwise, it will get thrown when user attempts to call service
            else:
                rospy.logwarn(f'WARNING: Timed out waiting for transponder ping service. This will throw an error when user tries to ping a transponder.')
        self.send_transponder_ping = rospy.ServiceProxy('ping_transponders', PingTransponders)

    def handle_ping_modem(self, req):
        if self.queued_ping_request is not None:
            rospy.logwarn(f'WARNING: Received a ping request while handling another! Returning as if we timed-out of this request!')
            return PingModemResponse(timed_out=True)

        try:
            self.queued_ping_response = None
            self.queued_ping_request = req
            cycle_duration = self.slot_duration_seconds * self.num_slots
            if cycle_duration < req.timeout_sec:
                rospy.logwarn(f'WARNING: ping_modem request timeout is longer than cycle_duration! Using req.timeout_sec: {req.timeout_sec}')
                max_timeout = req.timeout_sec
            else:
                max_timeout = cycle_duration
            # now wait until we have a self.queued_ping_response
            start_t = rospy.get_time()
            while not rospy.is_shutdown():
                if rospy.get_time() - start_t > (max_timeout):
                    rospy.logerr(f'ERROR: we have waited max_timeout: {max_timeout} and have not gotten a ping_response. Timing out..')
                    return PingModemResponse(timed_out=True)

                if self.queued_ping_response is None:
                    rospy.sleep(0.1)
                else:
                    break
        except:
            rospy.logerr(f'ERROR: threw exception in tdma::handle_ping_modem: {traceback.format_exc()}')
            return PingModemResponse(timed_out=True)
        else:
            rospy.loginfo(f'INFO: Received a ping_response from ping_modem via tdma: {self.queued_ping_response}')
            return self.queued_ping_response
        finally:
            self.queued_ping_request = None

    def config_slot_mask(self, slots=None, allow_empty=False):
        if slots is not None:
            slots = self.parse_slots_value(slots=slots)
            # user passed value(s) for set of slots to allow <slots-traffic> in
            slot_mask = [slot for slot in self.active_slots if slot in slots]
        else:
            # when nothing was passed for this slot type, by default, mask is allow == True
            slot_mask = [slot for slot in self.active_slots]

        try:
            assert set(slot_mask).issubset(set(self.active_slots))
            if not allow_empty:
                assert slot_mask != []
        except AssertionError:
            raise RuntimeWarning(f'WARNING: requested slot config: {slots} is incompatible with active slots! Valid slots fall within: {self.active_slots}')
        return slot_mask

    def reconfigure(self, config, level):
        if self.first_dynamic_reconf:
            self.first_dynamic_reconf = False
            # due to way rosparam of typle list doesn't properly map to param server in launch file
            # ..meaning the defaults set in the .cfg file will always use those values at first call
            # ..so to avoid having defaults set (when ros handles this by calling all non-list type ros params from param server),
            # ..set values from init in config
            config['skew_cycle_sec'] = self.cycle_start_time_offset_sec
            config['software_mute'] = self.software_mute
            config['ping_modem'] = self.ping_modems
            config['pings_per_slot'] = self.pings_per_slot
            config['ping_modem_src'] = self.ping_modem_src
            config['ping_modem_timeout_sec'] = self.ping_modem_timeout_sec
            config['ping_cdr'] = self.ping_cdr

            config['ping_transponders'] = self.ping_transponders  
            config['ping_transponder_a'] = self.ping_transponder_a 
            config['ping_transponder_b'] = self.ping_transponder_b 
            config['ping_transponder_c'] = self.ping_transponder_c 
            config['ping_transponder_d'] = self.ping_transponder_d

            config['num_slots'] = self.num_slots
            config['slot_duration_seconds'] = self.slot_duration_seconds
            config['guard_time_seconds'] = self.guard_time_seconds
            config['packet_length_seconds'] = self.packet_length_seconds
            config['miniframe_rate'] = self.miniframe_rate
            config['dataframe_rate'] = self.dataframe_rate

            rospy.logdebug(f'DEBUG: First dynamic_reconfigure call, syncing config from init OR post invalidate cache')
            return config

        rospy.logdebug("DEBUG: tdma_advanced_mac dynamic reconfigure request received.")
        rospy.logdebug(f"DEBUG: config: {config}")

        self.cycle_start_time_offset_sec = config['skew_cycle_sec']
        try:
            self.setup_active_slots(
                    # always making sure to sanitize dynamic reconfigure args
                    active_slots=self.active_slots,
                    num_slots=self.num_slots,
                    slot_duration_seconds=int(config['slot_duration_seconds']),
                    cycle_start_time=datetime.datetime.fromtimestamp(self.cycle_start_time.timestamp() + self.cycle_start_time_offset_sec),
                    guard_time_seconds=int(config['guard_time_seconds']),
                    packet_length_seconds=int(config['packet_length_seconds']),
                )
        except RuntimeWarning as e:
            rospy.logwarn(f'WARNING! This TDMA node was passed an invalid config:')
            rospy.logwarn(f'WARNING! {e}')
            rospy.logwarn(f'WARNING! The values for packet_length_seconds and guard_time_seconds have not been changed')
            rospy.logwarn(f'WARNING! packet_length_seconds:. requested[INVALID]: {config["packet_length_seconds"]}')
            rospy.logwarn(f'WARNING! guard_time_seconds:.... requested[INVALID]: {config["guard_time_seconds"]}')
            rospy.logwarn(f'WARNING! packet_length_seconds:..... current[VALID]: {self.packet_length_seconds}')
            rospy.logwarn(f'WARNING! guard_time_seconds:........ current[VALID]: {self.guard_time_seconds}')
            config['guard_time_seconds'] = self.guard_time_seconds
            config['packet_length_seconds'] = self.packet_length_seconds

        if config['software_mute'] != self.software_mute:
            self.software_mute = config['software_mute']
            rospy.set_param('~software_mute', self.software_mute)

        self.ping_modems = config['ping_modem']
        self.ping_modem_src = config['ping_modem_src']
        self.pings_per_slot = config['pings_per_slot']
        self.ping_cdr = config['ping_cdr']
        self.ping_modem_timeout_sec = config['ping_modem_timeout_sec'] 

        self.ping_transponders = config['ping_transponders'] 
        self.ping_transponder_a = config['ping_transponder_a']
        self.ping_transponder_b = config['ping_transponder_b']
        self.ping_transponder_c = config['ping_transponder_c']
        self.ping_transponder_d = config['ping_transponder_d']

        # update derived param cycle_duration in super
        self.cycle_duration = self.num_slots * self.slot_duration_seconds

        miniframe_rate, maximum_miniframe_bytes = self.miniframe_rate, self.maximum_miniframe_bytes
        if self.miniframe_rate != config['miniframe_rate']:
            miniframe_rate = config['miniframe_rate']
            maximum_miniframe_bytes = PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_MINI_RATE[miniframe_rate].default

        dataframe_rate, maximum_dataframe_bytes = self.dataframe_rate, self.maximum_dataframe_bytes
        if self.dataframe_rate != config['dataframe_rate']:
            dataframe_rate = config['dataframe_rate']
            maximum_dataframe_bytes = PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_DATA_RATE[dataframe_rate].default

        self.set_frame_rate(
                miniframe_rate=miniframe_rate,
                dataframe_rate=dataframe_rate,
                maximum_miniframe_bytes=maximum_miniframe_bytes,
                maximum_dataframe_bytes=maximum_dataframe_bytes,
            )

        return config

    def generate_tdma_advanced_status(self, basic_msg):
        msg = TdmaAdvancedStatus()
        msg.software_mute = self.software_mute
        msg.message = self.get_formatted_slot_ranges()

        msg.miniframe_rate = self.miniframe_rate
        msg.dataframe_rate = self.dataframe_rate
        msg.maximum_dataframe_bytes = self.maximum_dataframe_bytes
        msg.maximum_miniframe_bytes = self.maximum_miniframe_bytes
        msg.num_slots = self.num_slots
        msg.active_slots = self.active_slots
        msg.nav_slots = self.nav_slots
        msg.comms_slots = self.comms_slots

        msg.pings_per_slot = self.pings_per_slot if self.ping_modems else 0
        msg.ping_modem_src = self.ping_modem_src
        msg.ping_transponders = self.ping_transponders

        # set basic msg attrs
        for slot in basic_msg.__slots__:
            setattr(msg, slot, getattr(basic_msg, slot))
        return msg

    def handle_this_cycle_nav_pings(self, current_slot, remaining_active_seconds, sent_modem_pings, sent_transponder_ping):
        # we are active and have atleast enough time to send another packet
        # if we are allowed to do nav pings in this slot, handle that first
        nav_ping_duration_sec = 0
        update_last_tx = False

        if current_slot in self.nav_slots and (not sent_modem_pings or not sent_transponder_ping):
            # in active slot with nav pings allowed
            sent_modem_pings, sent_transponder_ping, nav_ping_duration_sec = self.send_nav_ping(remaining_active_seconds, sent_modem_pings, sent_transponder_ping)

        if nav_ping_duration_sec > 0:
            # sent nav pings, return
            update_last_tx = True
        return sent_modem_pings, sent_transponder_ping, update_last_tx

    def handle_queuing_packet(self, current_slot, remaining_active_seconds, sent_modem_pings, sent_transponder_ping):
        sent_modem_pings, sent_transponder_ping, update_last_tx = self.handle_this_cycle_nav_pings(current_slot, remaining_active_seconds, sent_modem_pings, sent_transponder_ping)
        if update_last_tx:
            # sent nav pings, return
            return sent_modem_pings, sent_transponder_ping, update_last_tx

        if current_slot in self.comms_slots:
            # check and make sure comms are allowed this slot
            # no nav pings sent this iter, comms allowed, go ahead with packet
            position_in_queue = self.send_next_packet()

            if position_in_queue is not None:
                update_last_tx = True
            # after sending comms packet, we clear the flags on nav pings
            # after sending all modem pings requested, a transponder ping (if requested) is sent out, 
            # .. then comms are allowed to use the remaining active time
            sent_modem_pings, sent_transponder_ping = False, False
            rospy.logdebug(f'DEBUG: tdma advanced sent packet, position in queue: {position_in_queue}')
            
        return sent_modem_pings, sent_transponder_ping, update_last_tx

    def on_slot_changed(self):
        self.sent_modem_pings, self.sent_transponder_ping = False, False

    def spin(self):
        rate = rospy.Rate(15)
        last_tx_time = 0
        msg = None

        while not rospy.is_shutdown():
            msg = self.get_current_slot_info(software_mute=self.software_mute)
            self.tdma_status_publisher.publish(self.generate_tdma_advanced_status(basic_msg=msg))

            if self.can_transmit_now(msg=msg, last_tx_time=last_tx_time):
                # we are active and have atleast enough time to send another packet
                self.sent_modem_pings, self.sent_transponder_ping, update_last_tx = self.handle_queuing_packet(
                        msg.current_slot,
                        msg.remaining_active_seconds,
                        self.sent_modem_pings, 
                        self.sent_transponder_ping
                    )
                if update_last_tx:
                    last_tx_time = rospy.get_time()
                else:
                    # nothing sent, sleep then check again
                    rospy.logdebug(f'DEBUG: Nothing sent during tdma... remaining_active_seconds: {msg.remaining_active_seconds}')
                    rate.sleep()
                    continue
            else:
                # not enough time. sleep for rate and try again
                rate.sleep()
                continue

    def send_nav_ping(self, remaining_active_seconds, sent_modem_pings, sent_transponder_ping):
        nav_ping_duration_sec = 0
        if self.queued_ping_request is not None:
            start_t = rospy.get_time()
            # handle in tdma ping_request
            req = self.queued_ping_request
            rospy.loginfo(f'Sending TWTT ping to modem_src: {req.dest}')
            self.queued_ping_response = self.send_modem_ping(req)
            self.queued_ping_request = None # for good measure so we don't send another ping with this request

            nav_ping_duration_sec = rospy.get_time() - start_t

        elif self.ping_modems and not sent_modem_pings:
            # fire off the modem pings
            start_t = rospy.get_time()
            for i in range(self.pings_per_slot):
                # before sending ping, make sure we have enough time remaining
                nav_ping_duration_sec = rospy.get_time() - start_t
                remaining_time = remaining_active_seconds - nav_ping_duration_sec
                if remaining_time < self.ping_modem_timeout_sec:
                    rospy.logwarn_throttle(self.slot_duration_seconds, f'WARNING: Not enough time to send another ping with remaining time: {remaining_time}s. Cancelled ping [{i + 1}/{self.pings_per_slot}]')
                    break

                req = PingModemRequest(dest=self.ping_modem_src, rate=self.miniframe_rate, cdr=self.ping_cdr, hexdata=bytes(), timeout_sec=self.ping_modem_timeout_sec)
                rospy.logdebug(f'[{i + 1}/{self.pings_per_slot}]: Sending TWTT ping to modem_src: {self.ping_modem_src}, request:\n{req}')
                if self.roslog_level < rospy.DEBUG: rospy.loginfo(f'[{i + 1}/{self.pings_per_slot}]: Sending TWTT ping to modem_src: {self.ping_modem_src}')

                resp = self.send_modem_ping(req)

                rospy.logdebug(f'DEBUG: Response from send_modem_ping:\n{resp}')
                if resp.timed_out:
                    if self.roslog_level < rospy.DEBUG: rospy.loginfo(f'INFO: Response from send_modem_ping TIMED OUT')

            sent_modem_pings = True
            nav_ping_duration_sec = rospy.get_time() - start_t

        elif self.ping_transponders and not sent_transponder_ping:
            # fire off transponder ping
            start_t = rospy.get_time()

            req = PingTranspondersRequest(transponder_dest_mask=[self.ping_transponder_a, self.ping_transponder_b, self.ping_transponder_c, self.ping_transponder_d], timeout_sec=self.transponder_reply_timeout_sec)
            rospy.logdebug(f"DEBUG: Sending transponder ping, request:\n{req}")
            if self.roslog_level < rospy.DEBUG: rospy.loginfo(f"INFO: Sending transponder ping, req.transponder_dest_mask: {req.transponder_dest_mask}, req.timeout_sec: {req.timeout_sec}")
            resp = self.send_transponder_ping(req)
            rospy.logdebug(f"DEBUG: Sent transponder ping. Response:\n{resp}")
            if self.roslog_level < rospy.DEBUG: rospy.loginfo(f"INFO: Sent transponder ping. resp.travel_times: {resp.travel_times}")
            sent_transponder_ping = True
            nav_ping_duration_sec = rospy.get_time() - start_t

        return sent_modem_pings, sent_transponder_ping, nav_ping_duration_sec


if __name__ == "__main__":
    try:
        tdma_state_node = TdmaAdvancedMacNode()
        rospy.loginfo("tdma_advanced_node shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("tdma_advanced_node shutdown (interrupt)")


