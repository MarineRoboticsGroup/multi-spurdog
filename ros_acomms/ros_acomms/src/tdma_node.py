#!/usr/bin/env python3

import rospy
from rospy.msg import AnyMsg
from collections import deque
from std_msgs.msg import Header, String
from ros_acomms_msgs.srv import QueueTxPacket, QueueTxPacketRequest
from ros_acomms_msgs.srv import GetNextPacketData
from ros_acomms_msgs.msg import Packet, TdmaStatus
import datetime
import dateutil.parser
import json
import logging
from threading import Thread

from mac_utils import PSK_ROS_FDP_Rates, ManualTransmitRequestQueue, TagConfig # pure python class for non-ros functionality
from mac_utils import validate_slot_range, parse_slot_tags

class TdmaMacNode(object):
    def __init__(self, subclass=False):
        if not subclass:
            rospy.init_node('tdma_mac')
            self.tdma_status_publisher = rospy.Publisher('~tdma_status' if rospy.get_param('~publish_private_status', False) else 'tdma_status', TdmaStatus, queue_size=3)

        self.setup_active_slots(
                active_slots=rospy.get_param('~active_slots', 0), # active_slots: (int)/[int,int,..] turns this param into list of ints, slots we're active in
                num_slots=rospy.get_param('~num_slots', 4),
                slot_duration_seconds=rospy.get_param('~slot_duration_seconds', 30),
                cycle_start_time=dateutil.parser.parse(rospy.get_param('~cycle_start_time', '2024-01-01T00:00:00Z')),
                guard_time_seconds=rospy.get_param('~guard_time_seconds', 5),
                packet_length_seconds=rospy.get_param('~packet_length_seconds', 5),
                always_send_test_data=rospy.get_param('~always_send_test_data', False)
            )

        self.set_frame_rate(
                miniframe_rate=rospy.get_param('~miniframe_rate', 1),
                dataframe_rate=rospy.get_param('~dataframe_rate', 5),
                maximum_miniframe_bytes=rospy.get_param('~maximum_miniframe_bytes', None),
                maximum_dataframe_bytes=rospy.get_param('~maximum_dataframe_bytes', None),
            )

        self.wait_for_services(timeout=rospy.get_param('~wait_for_services_timeout_sec', None))

        # Figure out current log level of this rosnode
        logger = logging.getLogger("rosout")
        self.roslog_level = logger.getEffectiveLevel()

        # IF CATXF messages are on, Use CATXF timestamp to signal when we are done transmitting
        rospy.Subscriber('txf', Header, self.on_txf)
        self.last_txf_time = rospy.get_time()

        # Manual Transmit Queue Setup
        #   - setup manual transmit queue for nmea_to_modem as example for how a subclass would add manual topics to this structure
        #   - subclasses can add manual_transmit_topics in init for the super class to manage
        self.manual_transmit_queue = ManualTransmitRequestQueue(maxlen=rospy.get_param('~maxlen_manual_queue', 1))
        # this is how sub classes should add manual transmit topics in their init
        self.manual_transmit_queue.add_manual_transmit_topic(topic='nmea_to_modem', msg_type=String)
        self.allow_manual_tx_during_priority_slots = rospy.get_param('~allow_manual_tx_during_priority_slots', False)

        # mechanism for registering callbacks in child classes on slot change
        self.on_slot_change_callbacks = []
        self.on_slot_change_thread = Thread(target=self.on_slot_change, daemon=True)
        self.on_slot_change_thread.start()

        self.init_link_layer_tags()

        if not subclass:
            rospy.loginfo("~*+ tdma_mac node running.")
            self.spin()

    def init_link_layer_tags(self):
        slot_tags_param = rospy.get_param('~slot_tags', None)
        self.slot_tags, slot_tags_param = parse_slot_tags(num_slots=self.num_slots, slot_tags_param=slot_tags_param)
        self.untagged_slots = []
        for slot in self.active_slots:
            if self.slot_tags[slot].minimum_priority or \
               self.slot_tags[slot].require_tags or \
               self.slot_tags[slot].exclude_tags:
                # if the slot that is in our set of active_slots has ANY tags...
                continue
            self.untagged_slots.append(slot)

        if not self.untagged_slots:
            rospy.logwarn(f'WARNING: There are NO untagged slots / dedicated slots!')
            rospy.logwarn(f'WARNING: ALL traffic must meet QoS tags for ALL slots!')
        else:
            rospy.loginfo(f'INFO: untagged_slots:\n{self.untagged_slots}')

        rospy.loginfo(f'INFO: ~slot_tags_param:\n{slot_tags_param}')
        rospy.loginfo(f'DEBUG: ~slot_tags:\n{self.slot_tags}')

    def register_on_slot_change_callback(self, callback):
        self.on_slot_change_callbacks.append(callback)

    def wait_for_services(self, timeout=120):
        rospy.loginfo("INFO: tdma waiting for queue_tx_packet service")
        rospy.wait_for_service('queue_tx_packet', timeout=timeout)
        self.queue_tx_packet = rospy.ServiceProxy('queue_tx_packet', QueueTxPacket)

        try:
            rospy.loginfo("INFO: tdma waiting for get_next_packet_data service")
            rospy.wait_for_service('get_next_packet_data', timeout=timeout)
        except rospy.exceptions.ROSException:
            rospy.logwarn(f'WARNING: get_next_packet_data ServiceProxy not up when tdma launched!')
        finally:
            self.get_next_packet_data = rospy.ServiceProxy('get_next_packet_data', GetNextPacketData)

    def expand_slot_range(self, slot_range):
        # will parse out a range string with start:stop:(optional)step
        # first see if we need to expand
        try:
            return [int(slot_range)]
        except (ValueError, TypeError):
            pass # we have a range string

        expanded_slots = []
        match = validate_slot_range(slot_range, last_valid_slot=self.num_slots - 1)
        if not match:
            err = f'ERROR: Invalid slot range passed: {slot_range}'
            rospy.logfatal(err)
            raise AttributeError(err) # if sub classes want to handle this differently, handle exc

        start, end, step = match
        if end <= start:
            # handle wrap around case
            for i in range(0, end + 1, step):
                expanded_slots.append(i)
            for i in range(start, self.num_slots, step):
                expanded_slots.append(i)
        else:
            for i in range(start, end + 1, step):
                expanded_slots.append(i)
        return expanded_slots

    def parse_slots_value(self, slots):
        if slots == [] or slots == '':
            return []

        expanded_slots = []
        if isinstance(slots, str):
            if ',' in slots:
                for slot in slots.split(','):
                    expanded_slots.extend(self.expand_slot_range(slot))
                return expanded_slots
            return [*self.expand_slot_range(slots)]
        elif isinstance(slots, list):
            for slot in slots:
                expanded_slots.extend(self.expand_slot_range(slot))
            return expanded_slots
        return [*self.expand_slot_range(slots)]

    def setup_active_slots(self, active_slots=None,
                                 num_slots=None,
                                 slot_duration_seconds=None,
                                 cycle_start_time=None,
                                 guard_time_seconds=None,
                                 packet_length_seconds=None,
                                 always_send_test_data=None):
        # update value if not None
        if num_slots is not None:
            self.num_slots = num_slots

        if active_slots != None:
            # turn string into single int or list of str
            try:
                assert self.parse_slots_value(slots=active_slots) != []
                self.active_slots = self.parse_slots_value(slots=active_slots)
            except AssertionError:
                # if we have NOT previously set active_slots, user is trying to pass an empty list for active_slots
                if 'active_slots' not in dir(self): 
                    raise AttributeError(f'FATAL: active_slots cannot be an empty list! active_slots: {repr(active_slots)}')
                rospy.logwarn(f'WARNING: active_slots cannot be empty list! Keeping prior active_slots: {self.active_slots}')
            except TypeError:
                raise
            except:
                err = f'FATAL: active_slots must be a single integer, list of integers or, a comma separated string of integers. active_slots: {active_slots}'
                rospy.logerr(err)
                raise TypeError(err)

        # make sure none of the active_slots are greater than the highest valid slot (num_slots - 1)
        invalid_active_slots = [slot for slot in self.active_slots if slot > self.num_slots - 1]
        if invalid_active_slots:
            err = f'FATAL: Invalid active_slots. out of bounds of num_slots. offending slots: {invalid_active_slots}. active_slots are zero-indexed. The highest/last valid slot is: {self.num_slots - 1}'
            rospy.logerr(err)
            raise AttributeError(err)

        self.slot_duration_seconds = slot_duration_seconds or self.slot_duration_seconds
        self.always_send_test_data = bool(always_send_test_data)
        self.cycle_start_time = cycle_start_time or self.cycle_start_time
        self.cycle_duration = self.num_slots * self.slot_duration_seconds
        self.cycle_start_secs = self.cycle_start_time.timestamp()

        # slot dict lookup, we check this as a step in determining if we are active
        self.slots = [{'active': i in self.active_slots} for i in range(self.num_slots)]

        # throw error if user configures a tdma that will never allow a transmit
        # .. in subclasses, if they want to allow this, just wrap this in a try-catch. TODO, possibly handle this differently
        if packet_length_seconds + guard_time_seconds >= self.slot_duration_seconds:
            raise RuntimeWarning(f'FATAL: Requested configuration will not allow tdma to transmit! The packet_length_seconds + guard_time_seconds MUST BE < slot_duration_seconds. {packet_length_seconds} + {guard_time_seconds} >= {self.slot_duration_seconds}')

        self.guard_time_seconds = guard_time_seconds if guard_time_seconds is not None else self.guard_time_seconds
        self.packet_length_seconds = packet_length_seconds if packet_length_seconds is not None else self.packet_length_seconds

    def set_frame_rate(self, miniframe_rate=None,
                             dataframe_rate=None,
                             maximum_miniframe_bytes=None,
                             maximum_dataframe_bytes=None):
        # update value if not None
        self.miniframe_rate = miniframe_rate or self.miniframe_rate
        self.dataframe_rate = dataframe_rate or self.dataframe_rate

        # update value when valid. if, invalid, gets set to default for the rate
        if hasattr(self, 'maximum_miniframe_bytes'):
            self.maximum_miniframe_bytes = maximum_miniframe_bytes or self.maximum_miniframe_bytes
        else:
            self.maximum_miniframe_bytes = maximum_miniframe_bytes or PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_MINI_RATE[self.miniframe_rate].default

        if hasattr(self, 'maximum_dataframe_bytes'):
            self.maximum_dataframe_bytes = maximum_dataframe_bytes or self.maximum_dataframe_bytes
        else:
            self.maximum_dataframe_bytes = maximum_dataframe_bytes or PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_DATA_RATE[self.dataframe_rate].default

    def get_valid_max_bytes_for_rate(self, rate, req_max_bytes, dataframe=True):
        if req_max_bytes is None:
            # invalid value. Will throw an exception shutting down this node
            if dataframe: err = f'Must set valid maximum_dataframe_bytes for rate {rate}. '
            else: err = f'Must set valid maximum_miniframe_bytes for rate {rate}. '
            err += f'For rates other than [1,3,5] there is no default. Using non FDP rate requires passing max_bytes!'
            rospy.logfatal(err)
            raise rospy.ROSInterruptException(err)
        #
        # handle max_bytes for data frames OR mini frames being too large for the data/mini rate
        #
        # create ref obj for the current (mini/data) rate to validate req_max_bytes
        # .. will return valid value for maximum_dataframe_bytes|maximum_miniframe_bytes.
        # .. sets rosparam for maximum_dataframe_bytes|maximum_miniframe_bytes if value passed was too large
        #
        if dataframe:
            fdp_max_bytes_4_rate = PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_DATA_RATE[rate]
        else:
            fdp_max_bytes_4_rate = PSK_ROS_FDP_Rates.FDP_MAX_BYTES_FOR_MINI_RATE[rate]

        if fdp_max_bytes_4_rate and req_max_bytes > fdp_max_bytes_4_rate.max_bytes:
            err = f'Cannot send a {req_max_bytes} byte '
            if dataframe:
                err += f'dataframe @ rate {rate}! Dataframe clipped to {fdp_max_bytes_4_rate.max_bytes} bytes'
                param_name = '~maximum_dataframe_bytes'
            else:
                err += f'miniframe @ rate {rate}! Miniframe clipped to {fdp_max_bytes_4_rate.max_bytes} bytes'
                param_name = '~maximum_miniframe_bytes'

            rospy.logerr(err)
            rospy.set_param(param_name, fdp_max_bytes_4_rate.max_bytes)
            rospy.logerr(f'rosparam server updated: {param_name}:{fdp_max_bytes_4_rate.max_bytes} @ rate: {rate}')
            return fdp_max_bytes_4_rate.max_bytes

        return req_max_bytes

    @property
    def maximum_miniframe_bytes(self):
        return self._maximum_miniframe_bytes

    @maximum_miniframe_bytes.setter
    def maximum_miniframe_bytes(self, val):
        self._maximum_miniframe_bytes = self.get_valid_max_bytes_for_rate(rate=self.miniframe_rate, req_max_bytes=val, dataframe=False)

    @property
    def maximum_dataframe_bytes(self):
        return self._maximum_dataframe_bytes

    @maximum_dataframe_bytes.setter
    def maximum_dataframe_bytes(self, val):
        self._maximum_dataframe_bytes = self.get_valid_max_bytes_for_rate(rate=self.dataframe_rate, req_max_bytes=val, dataframe=True)

    def on_txf(self, msg):
        # Reset our "time since last TX done" timer (used with guard time)
        self.last_txf_time = msg.stamp.to_sec()

    def get_time_to_next_dedicated_slot(self):
        current_slot, _ = self.get_current_slot()
        # the next dedicated slot is the slot where:
        #   - no require_tags, exclude_tags or, minimum_priority
        #   - that slot is in our list of active_slots
        for dedicated_slot in self.untagged_slots:
            if dedicated_slot >= current_slot:
                return self.get_remaining_time_to_slot(slot=dedicated_slot)

        if self.untagged_slots:
            # if we didn't return in the last loop but there are untagged slots,
            # .. the first untagged_slot is the next dedicated slot. this is the wrap around case
            return self.get_remaining_time_to_slot(slot=self.untagged_slots[0])
        # we have no untagged_slots
        return -1.0

    def get_remaining_time_to_slot(self, slot: int):
        # convenience method for child classes to use when querying,
        # .. the remaining time to a slot of interest (relative to the current_slot)
        if slot > self.num_slots - 1:
            rospy.logwarn(f'WARNING: Invalid slot passed to get_remaining_time_to_slot(slot={slot}), required: slot <= self.num_slots - 1, ({self.num_slots - 1})')
            return None

        current_slot, remaining_slot_seconds = self.get_current_slot()
        # unless we're in the slot of interest, we'll have atleast this much time remaining
        remaining_time_to_slot = remaining_slot_seconds

        if current_slot == slot:
            # we're in slot of interest
            return 0.0
        elif current_slot == self.num_slots - 1 and slot > 0:
            # we are at the last slot
            # calc full slots between current_slot and slot in seconds
            remaining_time_to_slot += self.slot_duration_seconds * slot
        elif current_slot < slot:
            # current slot is before slot of interest
            # count full slots starting *after the current_slot
            remaining_time_to_slot += self.slot_duration_seconds * ((slot - current_slot) - 1)
        elif current_slot > slot:
            # wrap around calculation, current_slot is after slot of interest
            # Need remaining seconds in cycle. then add seconds until slot of interest
            remaining_time_to_slot += self.slot_duration_seconds * ((self.num_slots - 1) - current_slot)
            # now add the slot cycles between 0 and the slot of interest (if slot=0, we don't add any time of course)
            remaining_time_to_slot += self.slot_duration_seconds * slot

        return remaining_time_to_slot

    def get_current_slot(self):
        now_secs = rospy.Time.now().to_sec()
        offset_secs = now_secs - self.cycle_start_secs
        in_cycle_secs = offset_secs % self.cycle_duration
        remaining_slot_seconds = self.slot_duration_seconds - (in_cycle_secs % self.slot_duration_seconds)

        current_slot = int(in_cycle_secs // self.slot_duration_seconds)
        return current_slot, remaining_slot_seconds

    def get_current_slot_info(self, software_mute=False):
        current_slot, remaining_slot_seconds = self.get_current_slot()
        we_are_active = self.slots[current_slot]['active'] and not software_mute
        
        if we_are_active:
            time_to_next_active = 0
            remaining_active_seconds = remaining_slot_seconds
        else:
            remaining_active_seconds = 0
            # Figure out how many seconds remain until we go active
            time_to_next_active = remaining_slot_seconds
            # plus any slot after this current one that isn't our active slot
            next_slot = current_slot + 1

            for _ in range(self.num_slots):
                # handle wrap around
                if next_slot >= self.num_slots: next_slot = 0 # wrapped around
                # add self.slot_duration_seconds for each slot that isn't our active slot between now and 
                # .. next active slot. break when we hit our active slot
                if next_slot not in self.active_slots:
                    time_to_next_active += self.slot_duration_seconds
                    next_slot += 1
                else:
                    break

        minimum_priority = self.slot_tags[current_slot].minimum_priority
        # populate a TdmaStatus ROS msg, to be published in spin
        msg = TdmaStatus(
                we_are_active=we_are_active, 
                current_slot=current_slot,
                remaining_slot_seconds=remaining_slot_seconds,
                remaining_active_seconds=remaining_active_seconds,
                time_to_next_active=time_to_next_active,
                time_to_next_dedicated_slot=self.get_time_to_next_dedicated_slot(),
                slot_duration_seconds=self.slot_duration_seconds,
                require_tags=list(self.slot_tags[current_slot].require_tags),
                exclude_tags=list(self.slot_tags[current_slot].exclude_tags),
                minimum_priority=minimum_priority if minimum_priority else 0,
            )
        msg.header.stamp = rospy.Time.now()

        return msg

    def can_transmit_now(self, msg, last_tx_time):
        if not msg.we_are_active:
            return False
        if msg.remaining_active_seconds < (self.guard_time_seconds + self.packet_length_seconds):
            # not enough time remaining in active slot for another packet transmit
            return False

        current_time = rospy.get_time()
        # check current_time with last_tx_time (tdma queued a packet in spin) + padding
        if current_time <= (last_tx_time + self.guard_time_seconds + self.packet_length_seconds) or \
           current_time <= self.last_txf_time + self.guard_time_seconds:
            # still too soon to transmit after previous tx, software lockout w/ guard_time_seconds
            return False

        return True

    def on_slot_change(self):
        rate = rospy.Rate(1)
        last_slot = None

        while not rospy.is_shutdown():
            rate.sleep()            
            
            current_slot, remaining_slot_seconds = self.get_current_slot()
            if last_slot is None:
                last_slot = current_slot

            if last_slot != current_slot:
                for callback in self.on_slot_change_callbacks:
                    try:
                        callback()
                    except:
                        rospy.logwarn(f'WARNING: Exception thrown during on_slot_change()')

                last_slot = current_slot
                rospy.sleep(self.slot_duration_seconds / 2)

    def spin(self):
        rate = rospy.Rate(5)
        last_tx_time = 0
        msg = None

        while not rospy.is_shutdown():
            msg = self.get_current_slot_info()
            self.tdma_status_publisher.publish(msg)

            if self.can_transmit_now(msg=msg, last_tx_time=last_tx_time):
                # we are active and have atleast enough time to send another packet
                position_in_queue = self.send_next_packet()
                last_tx_time = rospy.get_time()
                rospy.logdebug(f'DEBUG: tdma sent packet @ {last_tx_time}, position in queue: {position_in_queue}')
            else:
                # not enough time. sleep for rate and try again
                rate.sleep()
                continue

    def send_next_packet(self, insert_at_head=False, minimum_priority=None, slot_tags=None):
        # this will not send a manual request when minimum_priority is passed
        # .. unless ~allow_manual_tx_during_priority_slots is set to True (defaults to False)
        if minimum_priority is None or self.allow_manual_tx_during_priority_slots:
            if self.manual_transmit_queue:
                msg = self.manual_transmit_queue.handle_active_queue()
                if msg is not None:
                    rospy.logwarn(f'NOTICE: Sent manual transmit request instead of packet: {msg}')
                    return 0

        if slot_tags is None:
            # if not passed, use own slot_tags (TODO: might want to send a slot_tags other than own?)
            slot_tags = self.slot_tags

        current_slot, _ = self.get_current_slot()
        if minimum_priority is None:
            minimum_priority = slot_tags[current_slot].minimum_priority

        try:
            rospy.logdebug_throttle(1, f'DEBUG: current_slot: {current_slot} slot_tags[current_slot]: {slot_tags[current_slot]}')
            packet_data_response = self.get_next_packet_data(num_miniframe_bytes=self.maximum_miniframe_bytes,
                                                             num_dataframe_bytes=self.maximum_dataframe_bytes,
                                                             minimum_priority=minimum_priority,
                                                             require_tags=list(slot_tags[current_slot].require_tags),
                                                             exclude_tags=list(slot_tags[current_slot].exclude_tags))
        except rospy.service.ServiceException:
            rospy.logwarn(f'WARNING: send_next_packet() threw error on get_next_packet_data service callback!')
            return -1

        rospy.logdebug("DEBUG: packet_data_response {}".format(packet_data_response))

        if self.always_send_test_data:
            # fill the remaining space in the packet with zeros
            # see how many extra bytes we could pack into the miniframe and dataframe
            unallocated_bytes_miniframe = self.maximum_miniframe_bytes - len(packet_data_response.miniframe_bytes)
            unallocated_bytes_dataframe = self.maximum_dataframe_bytes - len(packet_data_response.dataframe_bytes)

            if unallocated_bytes_miniframe:
                packet_data_response.miniframe_bytes += bytes([0] * unallocated_bytes_miniframe)

            if unallocated_bytes_dataframe:
                packet_data_response.dataframe_bytes += bytes(([0] * unallocated_bytes_dataframe))

            rospy.logdebug(f'DEBUG: always_send_test_data=True and this packet is not full, padding this packet with zeros. num zeros added to miniframe: {unallocated_bytes_miniframe}, dataframe: {unallocated_bytes_dataframe}')

        elif packet_data_response.num_messages == 0:
            return

        rospy.loginfo(f"INFO: Queuing Packet for TX (dest {packet_data_response.dest}, " +
                      f"{len(packet_data_response.miniframe_bytes)} miniframe bytes @ rate {self.miniframe_rate}, " +
                      f"{len(packet_data_response.dataframe_bytes)} dataframe bytes @ rate {self.dataframe_rate})")

        queue_packet = Packet()
        queue_packet.dataframe_rate = self.dataframe_rate
        queue_packet.miniframe_rate = self.miniframe_rate
        queue_packet.dest = packet_data_response.dest
        queue_packet.miniframe_bytes = packet_data_response.miniframe_bytes
        queue_packet.dataframe_bytes = packet_data_response.dataframe_bytes

        req = QueueTxPacketRequest(insert_at_head=insert_at_head,
                                   packet=queue_packet)

        resp = self.queue_tx_packet(req)
        return resp.position_in_queue


if __name__ == "__main__":
    import traceback
    try:
        tdma_state_node = TdmaMacNode()
        rospy.loginfo("tdma_node shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("tdma_node shutdown (interrupt)")

    except:
        rospy.loginfo(f"tdma_node shutdown. Thrown unhandled exception: {traceback.format_exc()}")
