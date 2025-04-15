#!/usr/bin/env python3

import rospy
from ros_acomms_msgs.msg import TdmaSlottedAlohaStatus
from tdma_advanced_node import TdmaAdvancedMacNode

from ros_acomms.cfg import tdma_advancedConfig
from ros_acomms.cfg import tdma_slotted_alohaConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mac_utils import expanded_slots_as_ranges # pure python class for non-ros functionality

class TdmaSlottedAlohaMacNode(TdmaAdvancedMacNode):
    """
    An advanced TDMA MAC node that can execute a scripted test plan.

    Inherits from TdmaAdvancedMacNode.
    """
    def __init__(self, subclass=False):
        if not subclass:
            rospy.init_node('tdma_slotted_aloha_mac')
            self.tdma_status_publisher = rospy.Publisher('~tdma_slotted_aloha_status' if rospy.get_param('~publish_private_status', False) else 'tdma_slotted_aloha_status', TdmaSlottedAlohaStatus, queue_size=0)
        super().__init__(subclass=True)

        # configure aloha slots and get minimum priority for each
        self.aloha_slot_priority = rospy.get_param('~aloha_slot_priority', 0)
        # allow_aloha_slots_only if True, allows node to run with only group slots for this node.
        # in this mode, the node will only be able to send traffic in aloha slots shared with other nodes
        self.allow_aloha_slots_only = rospy.get_param('~allow_aloha_slots_only', False)
        self._aloha_slots_range = ''
        self.handle_set_aloha_slots(aloha_slots=rospy.get_param('~aloha_slots', []))

        if not subclass:
            self.first_dynamic_reconf = True
            self.first_dynamic_reconf_slotted_aloha = True
            self.reconfigure_super_class_server = DynamicReconfigureServer(type=tdma_advancedConfig, callback=self.reconfigure, namespace='advanced')
            self.reconfigure_server = DynamicReconfigureServer(type=tdma_slotted_alohaConfig, callback=self.reconfigure_slotted_aloha, namespace='slotted_aloha')

            try:
                rospy.loginfo(f'INFO: active_slots: {self._active_slots_range}')
                rospy.loginfo(f'INFO: comms_slots: {self._comms_slots_range}')
                rospy.loginfo(f'INFO: nav_slots: {self._nav_slots_range}')
                rospy.loginfo(f'INFO: aloha_slots: {self._aloha_slots_range}')
                
                rospy.logdebug(f'DEBUG: nav_slots expanded:\n{self.nav_slots}')
                rospy.logdebug(f'DEBUG: active_slots expanded:\n{self.active_slots}')
                rospy.logdebug(f'DEBUG: comms_slots expanded:\n{self.comms_slots}')                
                rospy.logdebug(f'DEBUG: aloha_slots expanded:\n{self.aloha_slots}')
            except:
                pass

            rospy.loginfo("~*+@@@ tdma_slotted_aloha_mac node running.")
            self.spin()

    def handle_set_aloha_slots(self, aloha_slots):
        try:
            self.aloha_slots = self.config_slot_mask(slots=aloha_slots)
        except RuntimeWarning:
            rospy.logwarn(f'NOTICE: Some or all aloha slots fall outside of the active_slots for this node.')
        else:
            if aloha_slots == self.aloha_slots:
                # if self.aloha_slots is not trimmed (matches aloha_slots) then all aloha_slots fall in active_slots
                rospy.logwarn(f'NOTICE: All aloha slots fall within active_slots.. Maybe this is intentional but, aloha_slots are meant to be shared')
            else:
                rospy.logwarn(f'NOTICE: Some aloha slots fall within active_slots some do not. Maybe this is intentional but, aloha_slots are meant to be shared')
        finally:
            self.aloha_slots = self.parse_slots_value(slots=aloha_slots)
            # make sure we have some slots of our own. Otherwise warn user or crash
            if set(self.active_slots).issubset(set(self.aloha_slots)):
                if not self.allow_aloha_slots_only:
                    rospy.logfatal(f'FATAL: All active_slots fall in aloha_slots.. This node does not have any slots of its own. Only shared aloha_slots.')
                    rospy.logfatal(f'WARNING: allow_aloha_slots_only={self.allow_aloha_slots_only} aloha_slots: {self.aloha_slots}')
                    raise RuntimeWarning(f'FATAL: All active_slots fall in aloha_slots.. This node does not have any slots of its own. Set allow_aloha_slots_only=True to override this behavior')
                else:
                    rospy.logwarn(f'WARNING: All active_slots fall in aloha_slots.. This node does not have any slots of its own! Only shared aloha_slots. allow_aloha_slots_only=True')
            # now cache the aloha slots range
            self._aloha_slots_range = expanded_slots_as_ranges(slots=self.aloha_slots)
            rospy.loginfo(f'INFO: aloha_slots: {self._aloha_slots_range}, active_slots: {self._active_slots_range}')
            
            if self.nav_slots != [] and self.aloha_slots != []:
                # check if nav_slots overlap with aloha slot so we can remove it TODO handle this better?
                if set(self.nav_slots).issubset(set(self.aloha_slots)):
                    # all the nav_slots overlap with aloha slots. clear nav_slots so we don't stomp on aloha slots with nav pings
                    if self.ping_modems or self.ping_transponders:
                        rospy.logfatal(f'FATAL: All nav_slots:({self._nav_slots_range}) overlap with aloha_slots:({self._aloha_slots_range}) and auto pings are on')
                        raise RuntimeWarning(f'FATAL: All nav_slots:({self._nav_slots_range}) overlap with aloha_slots:({self._aloha_slots_range}) and auto pings are on. Nav slots cannot overlap with aloha slots')
                    else:
                        rospy.logwarn(f'WARNING: All nav_slots:({self._nav_slots_range}) overlap with aloha_slots:({self._aloha_slots_range})!')
                        self.nav_slots = []
                        rospy.logwarn(f'WARNING: Cannot have nav slots during shared aloha slot. set nav_slots to [].')
                elif set(self.aloha_slots).intersection(set(self.nav_slots)):
                    # rospy.logwarn(f'WARNING: Some aloha_slots slots overlap with some nav_slots. Removing nav_slot(s) that fall in an aloha_slot (set of aloha_slots UNCHANGED): {self._aloha_slots_range}.symmetric_difference({self._nav_slots_range})')
                    # self.nav_slots = list(set(self.aloha_slots).symmetric_difference(set(self.nav_slots)))
                    rospy.logwarn(f'WARNING: Some aloha_slots slots overlap with some nav_slots. Removing nav_slot(s) that fall in an aloha_slot (set of aloha_slots UNCHANGED): {self._nav_slots_range}.difference({self._aloha_slots_range})')
                    self.nav_slots = list(set(self.nav_slots).difference(set(self.aloha_slots)))
                    rospy.logwarn(f'WARNING: Removed nav_slot(s) that fell within an aloha_slot(s): aloha_slots: {self._aloha_slots_range}, nav_slots: {self.nav_slots}')
            
            # call tdma advanced cache since we may have updated nav_slots
            self.cache_slot_ranges_for_status()
            rospy.loginfo(f'INFO: aloha_slots minimum priority: {self.aloha_slot_priority}')

    def reconfigure_slotted_aloha(self, config, level):
        if self.first_dynamic_reconf_slotted_aloha:
            self.first_dynamic_reconf_slotted_aloha = False
            config['aloha_slots_'] = ','.join([str(s) for s in self.aloha_slots])
            config['aloha_slot_priority'] = self.aloha_slot_priority
            rospy.logdebug(f'DEBUG: First dynamic_reconfigure call, syncing config from init')
            return config

        self.aloha_slot_priority = config['aloha_slot_priority']
        try:
            rospy.logwarn(f'NOTICE: attempting to update: aloha slots passed by dynamic_reconfigure: {config["aloha_slots_"]} if they differ from {self.aloha_slots}')
            if self.parse_slots_value(slots=config['aloha_slots_']) != self.aloha_slots:
                # new value for aloha slots, update and handle inavlid inputs
                self.handle_set_aloha_slots(aloha_slots=config['aloha_slots_'])
        except AttributeError:
            # raised when an invalid slot_range is passed
            rospy.logwarn(f'WARNING: aloha slots_ passed by dynamic_reconfigure has a malformed slot_range: {config["aloha_slots_"]}. Keeping old value: {self.aloha_slots}')
            config['aloha_slots_'] = ','.join([str(s) for s in self.aloha_slots])
        except:
            # not excpetion we handle in super class, don't use this value passed from dynamic reconfigure
            rospy.logwarn(f'WARNING: aloha slots_ passed by dynamic_reconfigure are invalid: {config["aloha_slots_"]}. Keeping old value: {self.aloha_slots}')
            config['aloha_slots_'] = ','.join([str(s) for s in self.aloha_slots])
        return config

    def in_aloha_slot(self, current_slot):
        return current_slot in self.aloha_slots

    def get_time_to_next_own_slot(self, current_slot):
        time_to_next_own_slot = None

        if current_slot in self.active_slots:
            # we are active in our own slot
            time_to_next_own_slot = 0.0
        elif current_slot == self.num_slots - 1:
            # wrapping around, in last slot
            time_to_next_own_slot = self.get_remaining_time_to_slot(slot=sorted(self.active_slots)[0])
        else:
            # see if current_slot is before any of our active slots
            for slot in self.active_slots:
                if current_slot < slot:
                    time_to_next_own_slot = self.get_remaining_time_to_slot(slot=slot)
                    break

            if time_to_next_own_slot is None:
                time_to_next_own_slot = self.get_remaining_time_to_slot(slot=sorted(self.active_slots)[0])

        return time_to_next_own_slot

    def get_time_to_next_aloha(self, current_slot):
        time_to_next_aloha = None
        if self.aloha_slots == []:
            return -1.0

        elif self.in_aloha_slot(current_slot=current_slot):
            # we're in an aloha slot right now
            return 0.0
        else:
            # not in aloha_slot currently
            if current_slot == self.num_slots - 1:
                # we're wrapped around, we're at the last slot, the next aloha_slot will be the first in the list
                time_to_next_aloha = self.get_remaining_time_to_slot(slot=sorted(self.aloha_slots)[0])
            else:
                # not in an aloha_slot currently, and not wrapped. 
                # figure out how many seconds till the next aloha slot
                for aloha_slot in sorted(self.aloha_slots):
                    if current_slot < aloha_slot:
                        # we're in a slot that is before [this] aloha_slot.
                        # [this] aloha_slot is the slot we are counting down to
                        time_to_next_aloha = self.get_remaining_time_to_slot(slot=aloha_slot)
                        break

                if time_to_next_aloha is None:
                    # we're wrapped around, past the last aloha_slot of this cycle, the next aloha_slot will be the first in the list
                    time_to_next_aloha = self.get_remaining_time_to_slot(slot=sorted(self.aloha_slots)[0])

        return time_to_next_aloha if time_to_next_aloha is not None else -1.0

    def generate_tdma_slotted_aloha_status(self, basic_msg, slotted_aloha_msg):
        advanced_msg = self.generate_tdma_advanced_status(basic_msg=basic_msg)
        slotted_aloha_msg.aloha_slots = self.aloha_slots
        slotted_aloha_msg.aloha_slot_priority = self.aloha_slot_priority
        for slot in advanced_msg.__slots__:
            setattr(slotted_aloha_msg, slot, getattr(advanced_msg, slot))

        slotted_aloha_msg.message = f'aloha_slots: {self._aloha_slots_range}, {slotted_aloha_msg.message}'
        # handle time_to_next for own slot and aloha slot
        slotted_aloha_msg.time_to_next_aloha = self.get_time_to_next_aloha(current_slot=basic_msg.current_slot)
        slotted_aloha_msg.time_to_next_own_slot = self.get_time_to_next_own_slot(current_slot=basic_msg.current_slot)

        # now set time_to_next_active with which ever comes first,
        if slotted_aloha_msg.time_to_next_own_slot < slotted_aloha_msg.time_to_next_aloha:
            # own slot coming up before next aloha, i.e., time_to_next_active is a count down to non group slot
            slotted_aloha_msg.time_to_next_active = slotted_aloha_msg.time_to_next_own_slot
        else:
            # aloha slot coming up next before our own slot, i.e., time_to_next_active is a count down to the group slot
            slotted_aloha_msg.time_to_next_active = slotted_aloha_msg.time_to_next_aloha
        return slotted_aloha_msg

    def handle_queuing_packet(self, current_slot, remaining_active_seconds, sent_modem_pings, sent_transponder_ping):
        sent_modem_pings, sent_transponder_ping, update_last_tx = self.handle_this_cycle_nav_pings(current_slot, remaining_active_seconds, sent_modem_pings, sent_transponder_ping)
        if update_last_tx:
            # sent nav pings, return
            return sent_modem_pings, sent_transponder_ping, update_last_tx

        position_in_queue = None
        # if aloha slot we pass the minimum_priority
        if self.in_aloha_slot(current_slot=current_slot):
            position_in_queue = self.send_next_packet(minimum_priority=self.aloha_slot_priority)
        elif current_slot in self.comms_slots:
            position_in_queue = self.send_next_packet()

        if position_in_queue is not None:
            update_last_tx = True
            # after sending comms packet, we clear the flags on nav pings
            # after sending all modem pings requested, a transponder ping (if requested) is sent out, 
            # .. then comms are allowed to use the remaining active time
            sent_modem_pings, sent_transponder_ping = False, False
            rospy.logdebug(f'DEBUG: tdma slotted aloha sent packet, position in queue: {position_in_queue}. Aloha transmit? {self.in_aloha_slot(current_slot=current_slot)}')

        return sent_modem_pings, sent_transponder_ping, update_last_tx

    def spin(self):
        rate = rospy.Rate(5)
        last_tx_time = 0
        msg = None
        slotted_aloha_msg = TdmaSlottedAlohaStatus()
        sent_modem_pings = False
        sent_transponder_ping = False

        while not rospy.is_shutdown():
            msg = self.get_current_slot_info(software_mute=self.software_mute)
            if self.in_aloha_slot(current_slot=msg.current_slot) and not msg.we_are_active:
                # aloha slots can be in active_slots list or not. this one is not in the active_slots list
                # make msg correct for an active slot if this aloha slot isn't in an active_slot
                msg.we_are_active = True
                msg.remaining_active_seconds = msg.remaining_slot_seconds
                msg.time_to_next_active = 0.0

            self.tdma_status_publisher.publish(self.generate_tdma_slotted_aloha_status(basic_msg=msg, slotted_aloha_msg=slotted_aloha_msg))

            if self.can_transmit_now(msg=msg, last_tx_time=last_tx_time):
                # we are active and have atleast enough time to send another packet
                sent_modem_pings, sent_transponder_ping, update_last_tx = self.handle_queuing_packet(
                        msg.current_slot,
                        msg.remaining_active_seconds,
                        sent_modem_pings, 
                        sent_transponder_ping
                    )
                if update_last_tx:
                    last_tx_time = rospy.get_time()
                else:
                    # nothing sent, sleep then check again
                    rate.sleep()
                    continue
            else:
                # not enough time. sleep for rate and try again
                rate.sleep()
                continue


if __name__ == "__main__":
    try:
        tdma_state_node = TdmaSlottedAlohaMacNode()
        rospy.loginfo("tdma_slotted_aloha_node shutdown (interrupt)")
    except rospy.ROSInterruptException:
        rospy.loginfo("tdma_slotted_aloha_node shutdown (interrupt)")


