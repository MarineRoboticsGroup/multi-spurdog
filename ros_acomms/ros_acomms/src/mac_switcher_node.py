#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Bool
from functools import partial
import threading
from ros_acomms_msgs.msg import MacSwitcherStatus, ManagedMacState

from mac_utils import ManagedMacNodeEntry

class MacSwitcherNode(object):
    """
    MAC switcher node for handling multiple MAC node instances
    Notes:
    - only 1 MAC node instance can be "active" (software_mute=False) at a time
    - trys to set ~publish_private_status on all managed MAC nodes but it's recommended that users set this to True to avoid topic overlap
    - this node waits until every managed MAC node has set it's ~software_mute in the parameter server
    - manual use of ~software_mute should still be supported for temporary mutes while nothing is manipulating the mac node BUT, it is now being modified by this node... YMMV
    - once we have all nodes up and a cached value for ~software_mute set by the node:
        - set ALL managed MAC nodes including the default_mac_namespace to MUTE to start
        - after all managed MAC nodes have been mutes, un-mute the default_mac_namespace

    ~default_mac_namespace: the namespace of the first active MAC type and the default fallback type
    ~managed_mac_namespaces: list of strings where the strings are the MAC namespaces IN THE SAME namespace as THIS node to manage.
    ~managed_mac_heartbeat_timeout_sec: 
    ~revert_on_deselect: 
    """
    def __init__(self):
        rospy.init_node('mac_switcher')
        # using publish_private_status option but stacking nested multi mac modes is not tested..
        self.mac_switcher_status_publisher = rospy.Publisher(
            '~mac_switcher_status' if rospy.get_param('~publish_private_status', False) else 'mac_switcher_status', MacSwitcherStatus, queue_size=5, latch=True
        )

        # transfer_lock when set, indicates NO MAC type change is in progress
        self.transfer_lock = threading.Event()
        self.transfer_lock.set()

        self.default_mac_namespace = rospy.get_param('~default_mac_namespace')
        self.managed_mac_namespaces = rospy.get_param('~managed_mac_namespaces', [])

        # when not set to -1, "selecting" a managed_mac_node requires that the node "selecting" do:
        # .. publish the same message used for "select" at minimum every managed_mac_heartbeat_timeout_sec OR, 
        # .. this node will revert to the default_mac_namespace after managed_mac_heartbeat_timeout_sec
        self.managed_mac_heartbeat_timeout_sec = rospy.get_param('~managed_mac_heartbeat_timeout_sec', -1)
        #
        # how to handle on_select() when turning off currently active MAC type
        # can either go back to default_mac_namespace or revert_last_mac
        # .. typically a user will just select the next MAC type they want to use rather than turning off the current type
        self.revert_on_deselect = rospy.get_param('~revert_on_deselect', False) # set to False by default because of edge case where you get in stuck state
        
        # setup managed nodes callbacks for setting software mute and selecting which mac node is active
        self.setup_managed_nodes()

        rospy.loginfo("~*+MMM mac_switcher node running.")
        self.spin()

    def on_heartbeat(self, msg, ns):
        if self._selected_mac['ns'] == ns:
            rospy.logwarn(f'NOTICE: Got heartbeat update for active_mac_ns! {ns}')
            self._selected_mac['last_heartbeat_rx'] = rospy.Time.now().secs
        else:
            rospy.logwarn(f'WARNING: Got heartbeat update for inactive mac ns... {ns}. No effect')

    def query_param_server_software_mute(self, ns, blocking=True, timeout=0.1):
        rospy.logdebug(f'WARNING: {"BLOCKING" if blocking else "NON-BLOCKING"} rosparam server query of: \"{rospy.get_namespace()}{ns}/software_mute\"')
        if not rospy.has_param(f'{ns}/software_mute'):
            # first try blocking or not
            rospy.sleep(timeout)
        
            while not rospy.is_shutdown():
                if not rospy.has_param(f'{ns}/software_mute'):
                    # second try
                    if not blocking:
                        return # if not blocking, won't try again. Return None
                    rospy.sleep(timeout)
                else:
                    break

        software_mute = rospy.get_param(f'{ns}/software_mute')
        rospy.loginfo(f'NOTICE: rosparam server query of: \"{rospy.get_namespace()}{ns}/software_mute\": {software_mute}')
        # if param server has it by now, cool return it
        return software_mute

    def create_select_callback(self, ns):
        return rospy.Subscriber(f'~{ns}/select', Bool, partial(self.on_select, ns=ns))

    def create_heartbeat_callback(self, ns):
        return rospy.Subscriber(f'~{ns}/heartbeat', Bool, partial(self.on_heartbeat, ns=ns))

    def create_software_mute_switch(self, ns):
        return rospy.Publisher(f'{ns}/set_software_mute', Bool, queue_size=5)

    def init_managed_mac_node_param_server(self, ns):
        # set publish_private_status:True
        rospy.set_param(f'{ns}/publish_private_status', True)
        # # was setting rospy.set_param(f'{ns}/software_mute', True) but then when node launches, it would get set

    def init_managed_mac_node_entry(self, ns, is_default=False):
        self.create_heartbeat_callback(ns=ns)
        return ManagedMacNodeEntry(
                ns=ns,
                is_default=is_default,
                set_software_mute=self.create_software_mute_switch(ns=ns),
                select_callback=self.create_select_callback(ns=ns),
            )

    def setup_managed_nodes(self):
        # loop over all the managed mac nodes and set the inital values in the parameter server right away
        [self.init_managed_mac_node_param_server(ns=ns) for ns in [*self.managed_mac_namespaces, self.default_mac_namespace]]
        # now for each managed mac namespace, create an entry and state. In the state, query the param server
        # setup for default here
        self.managed_mac_nodes = {
            'managed_node_entries': {
                self.default_mac_namespace: self.init_managed_mac_node_entry(
                    ns=self.default_mac_namespace,
                    is_default=True,
                ),
            },
            'managed_node_software_mute_state': {
                self.default_mac_namespace: self.query_param_server_software_mute(ns=self.default_mac_namespace, blocking=True)
            }
        }
        # start default muted
        rospy.sleep(2.0)
        self.mute(ns=self.default_mac_namespace)

        # configure dict for current selected mac and fallback/last
        self._selected_mac = {
            'ns': self.default_mac_namespace, 
            'software_mute': self.managed_mac_nodes['managed_node_software_mute_state'][self.default_mac_namespace],
            'last_heartbeat_rx': None,
        }
        # now that private params are set, use active_mac_ns.setter
        self.active_mac_ns = self.default_mac_namespace

        for ns in self.managed_mac_namespaces:
            self.managed_mac_nodes['managed_node_entries'][ns] = self.init_managed_mac_node_entry(ns=ns)
            # override_mute=True because we want to be sure the managed nodes are OFF and able to do software mute
            self.managed_mac_nodes['managed_node_software_mute_state'][ns] = self.query_param_server_software_mute(ns=ns, blocking=True)
            # mute each namespace to start
            rospy.sleep(2.0)
            self.mute(ns=ns)

        # pub first mac status after forcing default back to un-mute. The mac manager is up and all managed nodes are accounted for
        self.unmute()
        rospy.sleep(3.0) # need these sleeps to account for param server lag at initial boot up seq
        rospy.logwarn(f'NOTICE: mac_switcher is up, all managed mac nodes are up and nominal!')
        self.publish_mac_status()

    def set_software_mute(self, ns: str, mute: bool):
        try:
            self.managed_mac_nodes['managed_node_entries'][ns].set_software_mute.publish(Bool(data=mute))
            rospy.sleep(2.0)
            self.managed_mac_nodes['managed_node_software_mute_state'][ns] = self.query_param_server_software_mute(ns=ns, blocking=True)
        except KeyError:
            rospy.logerr(f'ERROR: mac_switcher set_software_mute request for a mac namespace we do not know about... {ns}')

    def mute(self, ns=None):
        if ns is None:
            ns = self.active_mac_ns

        rospy.loginfo(f'INFO: about to MUTE mac ns: {ns}')
        self.set_software_mute(ns=ns, mute=True)

    def unmute(self, ns=None):
        if ns is None:
            ns = self.active_mac_ns

        rospy.loginfo(f'INFO: about to UN-MUTE mac ns: {ns}')
        self.set_software_mute(ns=ns, mute=False)

    @property
    def active_mac_software_mute(self):
        # return self._selected_mac['software_mute']
        return self.managed_mac_nodes['managed_node_software_mute_state'][self.active_mac_ns]

    @property
    def active_mac_ns(self):
        return self._selected_mac['ns']

    @active_mac_ns.setter
    def active_mac_ns(self, value):
        ns = value
        if ns in self.managed_mac_nodes['managed_node_entries']:
            rospy.loginfo(f'INFO: setting new active_mac_ns: {ns}')
            # set last values before updating
            self._selected_mac['last_ns'] = self._selected_mac['ns']
            self._selected_mac['last_software_mute'] = self._selected_mac['software_mute']

            self._selected_mac['ns'] = ns
            self._selected_mac['last_heartbeat_rx'] = None
            self._selected_mac['software_mute'] = self.query_param_server_software_mute(ns=ns)
            # since we have an updated software mute, update class dict
            self.managed_mac_nodes['managed_node_software_mute_state'][ns] = self._selected_mac['software_mute']

    def revert_last_mac(self):
        # cache last mac type
        ns = self._selected_mac['last_ns']
        software_mute = self._selected_mac['last_software_mute']

        # set last to default before reverting to cached last
        self._selected_mac['last_ns'] = self.default_mac_namespace
        self._selected_mac['last_software_mute'] = self.query_param_server_software_mute(ns=self.default_mac_namespace, blocking=True)
        # since we have an updated software mute, update class dict
        self.managed_mac_nodes['managed_node_software_mute_state'][self.default_mac_namespace] = self._selected_mac['last_software_mute']

        # now set next mac to revert to
        rospy.logwarn(f'WARNING: reverting to the last mac type ns:{ns}, which was previously software_mute:{software_mute}')
        self.active_mac_ns = ns

    def handle_transfer(self, ns):
        # mute the active node
        self.mute()
        # call setter to update last_ and latest software mute
        self.active_mac_ns = ns
        # unmute the newly active node
        self.unmute()
        # publish status
        self.publish_mac_status()

    def publish_mac_status(self):
        managed_mac_namespaces = []
        for ns in self.managed_mac_nodes['managed_node_entries']:
            managed_mac_namespaces.append(ManagedMacState(ns=ns, software_mute=self.managed_mac_nodes['managed_node_software_mute_state'][ns]))

        self.mac_switcher_status_publisher.publish(
            MacSwitcherStatus(
                header=Header(stamp=rospy.Time.now()),
                default_mac_namespace=self.default_mac_namespace,
                managed_mac_namespaces=managed_mac_namespaces,
                active_mac_namespace=self.active_mac_ns,
                active_mac_software_mute=self.active_mac_software_mute,
                managed_mac_heartbeat_age=-1.0 if not self._selected_mac['last_heartbeat_rx'] else float(rospy.Time.now().secs - self._selected_mac['last_heartbeat_rx']),
                managed_mac_heartbeat_timeout_sec=self.managed_mac_heartbeat_timeout_sec,
                mac_namespace_on_timeout=self._selected_mac['last_ns'] if self.revert_on_deselect else self.default_mac_namespace,
            )
        )

    def on_select(self, msg, ns):
        try:
            if self.transfer_lock.wait():
                self.transfer_lock.clear() # now any subsequent call to on_select() will block

                if self.active_mac_ns == ns:
                    # SELECT/UNSELECT currently active mac type
                    if not msg.data:
                        if self.revert_on_deselect:
                            self.revert_last_mac()
                            self.handle_transfer(ns=self.active_mac_ns)
                        else:
                            # back to default mac type
                            self.handle_transfer(ns=self.default_mac_namespace)

                elif msg.data:
                    # SELECT a mac type that is NOT the currently active type
                    # make sure we can switch to the requested type
                    if ns in self.managed_mac_nodes['managed_node_entries']:
                        self.handle_transfer(ns=ns)
        finally:
            # always clear the event obj for the next call to on_select()
            self.transfer_lock.set()

    def spin(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            # check if we are in a managed_mac namespace with a managed_mac_heartbeat_timeout_sec
            if self.managed_mac_heartbeat_timeout_sec > 0:
                # we potentially need to enforce a hearbeat
                if self._selected_mac['ns'] != self.default_mac_namespace:
                    # we are using a managed mac namespace not the default AND this mac_switcher have a dead man switch
                    # .. for non default managed_mac_namespaces
                    if self._selected_mac['last_heartbeat_rx'] is None:
                        # set it once so we can timeout after this point if we don't get an update
                        self._selected_mac['last_heartbeat_rx'] = rospy.Time.now().secs

                    if (rospy.Time.now().secs - self._selected_mac['last_heartbeat_rx']) > self.managed_mac_heartbeat_timeout_sec:
                        # we have timeout! call select for default mac
                        rospy.logwarn(f'WARNING: MAC Switcher DEAD MAN SWITCH for mac: {self._selected_mac["ns"]}')
                        rospy.logwarn(f'WARNING: MAC Switcher DEAD MAN SWITCH reverting to default: {self.default_mac_namespace}')
                        self.on_select(msg=Bool(data=True), ns=self.default_mac_namespace)
                        rospy.logwarn(f'WARNING: MAC Switcher DEAD MAN SWITCH reverted back to default: {self._selected_mac["ns"]}')

                    # publish the status while we are potentially counting down on non default mac
                    self.publish_mac_status()

            rate.sleep()

if __name__ == "__main__":
    try:
        mac_switcher_node = MacSwitcherNode()
        rospy.loginfo("mac_switcher_node shutdown (interrupt)")
    except rospy.ROSInterruptException:
        rospy.loginfo("mac_switcher_node shutdown (interrupt)")
