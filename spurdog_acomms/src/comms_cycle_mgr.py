#!/usr/bin/env python3
import rospy
import numpy as np
from datetime import datetime
import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
from ros_acomms_msgs.msg import(
    TdmaStatus
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)
from spurdog_acomms.msg import(
    InitPrior, PartialGraph, CommsCycleStatus
)
from spurdog_acomms.srv import(
    PreintegrateIMU, PreintegrateIMUResponse
)
from spurdog_acomms_utils.setup_utils import(
    configure_modem_addresses,
    configure_cycle_targets
)
from spurdog_acomms_utils.codec_utils import (
    encode_init_prior_data_as_int,
    encode_partial_graph_pose_as_int,
    encode_partial_graph_range_as_int,
    check_partial_graph_msg,
)
from spurdog_acomms_utils.nmea_utils import (
    parse_nmea_sentence,
    parse_nmea_cacmd,
    parse_nmea_cacma,
    parse_nmea_cacmr,
    parse_nmea_carfp,
    parse_nmea_carev
)
from spurdog_acomms_utils.graph_utils import (
    correct_first_relative_pose_for_failed_cycle,
)

class CycleManager:
    """This is a node to run the comms cycle for the vehicle."""
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.local_address = int(rospy.get_param("modem_address", 0))
        self.num_agents = int(rospy.get_param("num_agents", 1))
        self.num_landmarks = int(rospy.get_param("num_landmarks", 0))
        self.landmarks = rospy.get_param("landmarks", {}) # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = float(rospy.get_param("sound_speed", 1500))
        # Variables for addressing
        self.modem_addresses = {}
        self.cycle_target_mapping = {}
        # Variables for tdma and cycle execution
        self.tdma_status = TdmaStatus()
        self.tdma_cycle_sequence = 0
        self.msg_preload = 10 # seconds to allow for encoding the message before the tdma window opens
        self.msg_transmit_duration = 5 # seconds to allow for the message to be sent in the first half of the slot
        self.skew_buffer = 2 # seconds on either side to account for any skew in the modem clock
        # Variables for acomms event topic
        self.ping_method = "ping with payload"
        self.ping_timeout = 4 # seconds
        self.ping_slot_open = False
        self.loaded_msg = False # last message we sent to the modem
        # Variables for external sensors
        self.gps_fix = [[1,2,3],[0,0,0,1],[np.eye(6)]] # [position, orientation, covariance]
        # Variables for message handling
        self.staged_init_prior = InitPrior()
        self.staged_partial_graph = PartialGraph()
        self.partial_graph_data = {}
        self.cycle_graph_data = {}
        self.inbound_init_priors = {}
        self.inbound_partial_graphs = {}
        self.pose_time_lookup = {}
        self.in_water = False
        self.smooth_poses = False
        self.init_complete = False
        self.failed_cycle_relative_pose = {
            "key1": None,
            "key2": None,
            "position": None,
            "orientation": None,
            "sigmas": None
        }
        # Check services
        rospy.loginfo("[%s] Waiting for services..." % rospy.Time.now())
        rospy.wait_for_service("modem/ping_modem")
        self.ping_client = rospy.ServiceProxy("modem/ping_modem", PingModem)
        rospy.wait_for_service("preintegrate_imu")
        self.preintegrate_imu = rospy.ServiceProxy("preintegrate_imu", PreintegrateIMU)
        rospy.loginfo("[%s] Services ready, initializing topics" % rospy.Time.now())
        # Initialize topics
        self.tdma_from_modem = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_status, queue_size=1)
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        # Establish the message subs and pubs
        self.init_prior_pub = rospy.Publisher("modem/to_acomms/init_prior", InitPrior, queue_size=1)
        self.init_prior_bypass_pub = rospy.Publisher("modem/from_acomms/init_prior", InitPrior, queue_size=1)
        self.partial_graph_pub = rospy.Publisher("modem/to_acomms/partial_graph", PartialGraph, queue_size=1)
        self.partial_graph_bypass_pub = rospy.Publisher("modem/from_acomms/partial_graph", PartialGraph, queue_size=1)
        # Initialize Subscribers for handling external sensors
        self.gps = rospy.Subscriber("gps", PoseWithCovarianceStamped, self.on_gps)
        self.mission_status = rospy.Subscriber("mission_state", String, self.on_mission_state)
        self.comms_cycle_status_pub = rospy.Subscriber("comms_cycle_status", CommsCycleStatus, self.on_comms_cycle_status)
        # Initialize the modem addresses and cycle targets
        rospy.loginfo("[%s] Topics ready, initializing comms cycle" % rospy.Time.now())
        self.configure_comms_cycle()
        # Start the cycle
        rospy.loginfo("[%s] Comms Cycle Configured" % rospy.Time.now())

    def configure_comms_cycle(self):
        """This function configures the comms cycle for the vehicle.
        """
        # Get the modem addresses and cycle targets
        self.modem_addresses = configure_modem_addresses(self.num_agents, self.num_landmarks, self.local_address)
        self.cycle_target_mapping = configure_cycle_targets(self.modem_addresses)
        return

    def on_comms_cycle_status(self, msg):
        """This function handles the comms cycle status message
        Args:
            msg (CommsCycleStatus): The comms cycle status message
        """
        # Get message field:
        cycle_seq_number = msg.sequence_number
        self.init_complete = msg.init_complete
        cycle_complete = msg.cycle_complete
        self.smooth_poses = msg.should_smooth
        if msg.should_smooth:
            self.failed_cycle_relative_pose = {
                "key1": msg.key1,
                "key2": msg.key2,
                "position": msg.translation,
                "orientation": msg.quaternion,
                "sigmas": msg.sigmas
            }
            #NOTE: Smoothing might give A0-A5, but message format allows for A0-A1.
            # To remedy this, without modifying the pose sequencing or error checking held here,
            # we have to assume that the partial graphs are always connected to the previous
            # regardless of what the indices are.
            # So while the A0-A5 relative pose is incorporated into the between labeled A5-A6,
            # it will actually connect A0-A6
            # This can be can be corrected easily on the graph management side, and is safe
            # because we're preventing publication of unconnected graphs here.
        else:
            self.failed_cycle_relative_pose = {
                "key1": None,
                "key2": None,
                "position": None,
                "orientation": None,
                "sigmas": None
            }
        return

    def on_tdma_status(self, msg):
        """This function updates the modem TDMA status
        Args:
            msg (TdmaStatus): The TDMA status message
        """
        # Get the TDMA status
        current_slot = msg.current_slot
        we_are_active = msg.we_are_active
        remaining_sec_in_slot = msg.remaining_slot_seconds
        remaining_active_sec = msg.remaining_active_seconds
        time_to_next_active = msg.time_to_next_active
        slot_duration = msg.slot_duration_seconds
        elapsed_time_in_slot = slot_duration - remaining_sec_in_slot
        #rospy.logwarn("[%s] TDMA Msg: Slot: %s, Prev Slot: %s" % (msg_id, current_slot, self.tdma_status.current_slot))
        # Check if the TDMA cycle is restarting:
        if current_slot == 0 and current_slot != self.tdma_status.current_slot:
            self.tdma_cycle_sequence += 1
            self.on_tdma_cycle_reset()

        # If we need to send messages to other modems:
        if self.num_agents > 1 or self.num_landmarks > 0:
        # Load the cycle message into the queue so its ready to go when we are active
            if time_to_next_active < self.msg_preload and not self.loaded_msg:
                if self.tdma_cycle_sequence == 0:
                    rospy.loginfo("[%s] Publishing Init Prior" % rospy.Time.now())
                    # Publish to the modem:
                    self.init_prior_pub.publish(self.staged_init_prior)
                    # Bypass the modem and publish directly to the graph manager
                    self.init_prior_bypass_pub.publish(self.staged_init_prior)
                    self.loaded_msg = True
                else:
                    rospy.loginfo("[%s] Publishing Partial Graph" % rospy.Time.now())
                    # Publish to the modem
                    self.partial_graph_pub.publish(self.staged_partial_graph)
                    # Bypass the modem and publish directly to the graph manager
                    self.partial_graph_pub.publish(self.staged_partial_graph)
                    self.loaded_msg = True
            # If we are active, we need to execute the ping cycle
            elif we_are_active == True:
                if elapsed_time_in_slot < 1:
                    rospy.loginfo("[%s] TDMA Active Slot Started, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
                    self.loaded_msg = False
                # NOTE: from t=0 to t=msg_transmit_duration, the modem is attempting to clear the queue
                # From t=msg_transmit_duration to t=ping_timeout + skew_buffer, the modem is sending the message
                elif elapsed_time_in_slot > self.msg_transmit_duration and remaining_sec_in_slot > (self.skew_buffer+self.ping_timeout):
                    # NOTE: This ensures that the ping cycle takes 5sec to clear the queue at the beginning of the slot, then 4sec for one ping,
                    # and 4sec for the second ping, then idle for 2sec to prevent overlap
                    self.execute_ping_cycle(current_slot, elapsed_time_in_slot)
                else:
                    pass
            else:
                # If we are not active, do nothing
                pass
        else:
            # This is an escape to prevent the ping cycle from executing if we have no other agents or landmarks
            rospy.logwarn("[%s] No other agents or landmarks to ping" % rospy.Time.now())

        # Regardless, update TDMA
        self.tdma_status = msg
        # self.tdma_status.current_slot = current_slot
        # self.tdma_status.we_are_active = we_are_active
        # self.tdma_status.remaining_slot_seconds = remaining_sec_in_slot
        # self.tdma_status.remaining_active_seconds = remaining_active_sec
        # self.tdma_status.time_to_next_active = time_to_next_active
        # self.tdma_status.slot_duration_seconds = slot_duration
        return

    def on_tdma_cycle_reset(self):
        """This function is called when the TDMA slot returns to 0
        """
        if self.init_complete:
            # NOTE: The smoothed relative pose is incorporated within build_partial_graph
            self.staged_partial_graph = self.build_partial_graph()
            # NOTE: If the graph fails the check, this will be an empty message
            # If there are no successful pings, the graph will be empty and preintegration won't occur
            # The checking function evaluates if the graph is connected, and without duplicates
            # It also check sfor gross errors (like violating the size limit)
            self.partial_graph_data.clear()
        elif self.staged_init_prior == None:
            # If we are not initialized, we need to build the init prior
            self.staged_init_prior = self.build_init_prior()
        else:
            self.partial_graph_data.clear()
            rospy.logwarn("[%s] TDMA Cycle Reset, waiting for init prior data" % rospy.Time.now())
        self.tdma_cycle_sequence += 1
        return

    def execute_ping_cycle(self, current_slot, elapsed_time_in_slot):
        """This function executes the cycle for the current TDMA slot
        - Assumes nothing about the tdma structure other than duration >10sec (limited by ros_acomms/tdma)
        """
        # Get the targets from the cycle target mapping
        first_tgt, second_tgt = self.cycle_target_mapping[current_slot]

        # Execute the first of two ping slots
        if first_tgt == []:
            rospy.logwarn("[%s] No targets to ping in this slot" % rospy.Time.now())
            return
        else:
            if elapsed_time_in_slot > 2:
                self.ping_slot_open = True
            elif elapsed_time_in_slot < 3 and self.ping_slot_open == True:
                self.send_ping(first_tgt)
                self.ping_slot_open = False
            else:
                return

        # Execute the second of two ping slots
        if second_tgt == []:
            rospy.logwarn("[%s] No second target to ping in this slot" % rospy.Time.now())
            return
        else:
            if elapsed_time_in_slot > 2 + self.ping_timeout:
                self.ping_slot_open = True
            elif elapsed_time_in_slot < (3 + self.ping_timeout):
                self.send_ping(second_tgt)
                self.ping_slot_open = False
            else:
                return
        return

    # Ping Handling:
    def on_nmea_from_modem(self, msg):
        """This function receives NMEA messages from the modem
        """
        nmea_type, data = parse_nmea_sentence(msg.data)
        # Process the NMEA data by field
        if nmea_type == "$CACMD": # Modem-to-host acknowledgement of a ping command
            src, dest = parse_nmea_cacmd(data)
            if data[0] == "PNG" and src == self.local_address:
                rospy.loginfo("[%s] Sent Ping to %s" % (rospy.Time.now(), chr(ord("A") + dest)))
            else:
                rospy.logerr("[%s] Received $CACMD with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMA": # Modem-to-host acknowledgement of a ping recieved
            src, dest, recieved_ping_time = parse_nmea_cacma(data)
            if data[1] == "PNG" and dest == self.local_address:
                self.request_preintegration(recieved_ping_time, True) # Request a relative pose measurement
                rospy.loginfo("[%s] Received Ping from %s" % (recieved_ping_time, chr(ord("A") + src)))
            elif data[1] == "PNG":
                rospy.loginfo("[%s] Overheard Ping from %s to %s" % (recieved_ping_time, chr(ord("A") + src), chr(ord("A") + dest)))
            else:
                rospy.logerr("[%s] Received $CACMA with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMR": # Modem-to-host acknowledgement of a ping response
            src, dest, recieved_ping_time = parse_nmea_cacmr(data)
            if data[1] == "PNR" and src == self.local_address:
                rospy.loginfo("[%s] Received Ping Response from %s" % (recieved_ping_time, chr(ord("A") + dest)))
            elif data[1] == "PNR":
                rospy.loginfo("[%s] Overheard Ping Response from %s to %s" % (recieved_ping_time, chr(ord("A") + src), chr(ord("A") + dest)))
            else:
                rospy.logerr("[%s] Received $CACMR with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CARFP" and data[5] == "-1": # Modem-to-host acknowledgement of a minipacket ping payload
            src, dest, recieved_msg_time, num_frames, payload = parse_nmea_carfp(data)
            if not recieved_msg_time or not src or not dest or not payload:
                rospy.logerr("[%s] CARFP message is missing required fields" % rospy.Time.now())
                return
            elif dest == self.local_address:
                rospy.loginfo("[%s] Received Ping from %s with payload %s" % (recieved_msg_time, chr(ord("A") + src), payload))
            elif dest != self.local_address:
                rospy.logerr("[%s] Overheard Ping-related $CARFP from %s to %s with paylaod %s" % (recieved_msg_time, chr(ord("A") + src), chr(ord("A") + dest), payload))
            else:
                rospy.logerr("[%s] Received $CARFP with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CAREV" and self.ping_method == None: # Modem-to-host $CAREV message to determine the firmware version
            firmware_version = parse_nmea_carev(data)
            if firmware_version[0] == "3":
                # New deckbox firmware
                self.ping_method = "ping with payload"
            else:
                # Old deckbox firmware
                self.ping_method = "no payload"
        else:
            return
        return

    def send_ping(self, target_addr):
        """This function sends a ping to the modem
        Args:
            target_addr (int): the target address
            symbol (str): the local key "A1" to put in the payload
        """
        # Get the next symbol for the ping payload
        symbol = chr(ord("A") + self.local_address) + str(self.modem_addresses[chr(ord("A") + self.local_address)][1]+1)
        # Set the ping request parameters
        ping_req = PingModemRequest()
        ping_req.dest = target_addr
        ping_req.rate = 1
        ping_req.cdr = 0
        # Check the modem version and if it supports the ping payload
        if self.ping_method == "ping with payload":
            ping_req.hexdata = bytearray(symbol.encode("utf-8"))
        else:
            # TODO: Add the capability to pass the hex data as a separate packet
            rospy.logwarn("[%s] Old deckbox firmware detected, sending ping without payload" % rospy.Time.now())
        ping_req.timeout_sec = self.ping_timeout

        # Attempt the ping:
        try:
            #rospy.loginfo("[%s] One Ping Only Vasily." % (rospy.Time.now()))
            rospy.loginfo("[%s] Sending Ping to %s with payload %s" % (rospy.Time.now(), target_addr, symbol))
            ping_resp = self.ping_client(ping_req)
            # Check if the ping timed out
            if ping_resp.timed_out:
                rospy.logwarn("[%s] Ping to %s Timed Out" % (rospy.Time.now(), target_addr))
            else:
                rospy.loginfo("[%s] Ping to %s Successful" % (rospy.Time.now(), target_addr))
                src = ping_resp.cst.src
                dest = ping_resp.cst.dest
                owtt = ping_resp.one_way_travel_time
                measured_range = owtt * self.sound_speed
                timestamp = ping_resp.timestamp
                # TODO: Verify this is rostime
                self.request_preintegration(timestamp, True) # Request a relative pose measurement
                # Search the modem_addresses for the dest address to get the dest_chr
                if dest < self.num_agents:
                    dest_chr = chr(ord("A") + dest)
                else:
                    dest_chr = "L%d" % (dest - self.num_agents)
                # Add to the cycle graph data
                graph_id = "RNG_%s_%s" % (symbol, dest_chr)
                self.partial_graph_data[graph_id] = {
                    "key1": symbol,
                    "key2": dest_chr,
                    "range": measured_range,
                }
        except rospy.ServiceException as e:
            rospy.logerr("[%s] Ping Service Call Failed: %s" % (rospy.Time.now(), e))
        return

    # Sensor data handling
    def request_preintegration(self, tj, adv_pose: bool = True):
        """This function requests a relative pose measurement from imu sensor handler node
        - If called with adv_pose = True, it logs the relative pose in partial graph data
        - If called with adv_pose = False, it clears the preintegration queue setting up
        a new cycle.
        Args:
            tj (rostime): The time of the new pose to mark
            adv_pose (bool): Whether to advance the pose index or just preintegrate to clear the queue
        """
        # Attempt to get a relative pose between the time provided and the last time we tried this
        local_chr = chr(ord("A") + self.local_address)
        key1_index = self.modem_addresses[local_chr][1]
        ti = self.pose_time_lookup[local_chr + str(key1_index)]
        try:
            response = self.preintegrate_imu(ti, tj)
            if not response.success:
                rospy.logerr(f"Preintegration failed: {response.error_message}")
                return None
            else:
                rospy.loginfo(f"Received preintegrated pose between {ti} and {tj}")
                if adv_pose:
                    # Advance the key indices and the time
                    key1 = local_chr + str(key1_index)
                    key2 = local_chr + str(key1_index+1)
                    self.modem_addresses[local_chr][1] = key1_index + 1
                    self.pose_time_lookup[key2] = tj
                    x_ij = response.pose_delta
                    pose = x_ij.pose.pose
                    position = np.array([pose.position.x, pose.position.y, pose.position.z])
                    orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                    covariance = np.array(response.pose_delta.covariance).reshape((6, 6))
                    sigmas = np.sqrt(np.diag(covariance))
                    # Add to the cycle graph data
                    graph_id = "BTWN_%s_%s" % (key1, key2)
                    self.partial_graph_data[graph_id] = {
                        "key1": key1,
                        "key2": key2,
                        "position": np.array([position.x, position.y, position.z]),
                        "orientation": np.array([orientation.x, orientation.y, orientation.z, orientation.w]),
                        "sigmas": sigmas
                    }
                else:
                    # This allows for calling preintegration to clear the queue without advancing the pose
                    pass
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return

    def on_gps(self, msg: PoseWithCovarianceStamped):
        """This function receives the GPS data from the estimator
        Args:
            msg (PoseWithCovarianceStamped): The GPS data
        """
        # Get the current time
        pose_time = msg.header.stamp
        # Replace the initial_position with the GPS data
        self.gps_fix[0] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.gps_fix[1] = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.gps_fix[2] = np.sqrt(np.diag(msg.pose.covariance))  # GPS covariance is a 6x6 matrix, we only need the diagonal
        # Log the reciept
        rospy.loginfo("[%s] Received GPS data" % pose_time)
        return

    def on_mission_state(self, msg: String):
        """This function receives the navigation state data from the estimator
        Args:
            msg (PoseStamped): The navigation state data
        """
        # Replace the initial_position with the GPS data
        # Split the message into a list
        # Get the time
        msg_time = rospy.Time.now()
        msg = msg.data.split("=")
        # Check if the message is valid
        if msg[0] == "IN_WATER":
            value = msg[1]
            if value == "true":
                if self.in_water == False:
                    # Clear preintegration queue
                    self.request_preintegration(msg_time, False)
                    # Build the initial prior factor msg
                    self.build_init_prior()
                    self.in_water = True
                    rospy.loginfo("[%s] In water, building init prior" % (rospy.Time.now()))
                else:
                    return
            elif value == "false":
                self.in_water = False
        else:
            return
        # Log the reciept
        rospy.loginfo("[%s] Changed water status to %s" % (rospy.Time.now(), msg.data))
        return

    # Message Handling Functions
    def build_init_prior(self):
        """This function sends an initial prior factor to the estimator
        Args:
            target_addr (int): the target address
        """
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1])
        # Encode the initial prior factor data into a message
        initial_position, initial_orientation, initial_sigmas = encode_init_prior_data_as_int(self.gps_fix[0], self.gps_fix[1], self.gps_fix[2])
        # Set the initial prior factor message
        init_prior_msg = InitPrior()
        init_prior_msg.local_addr = int(self.local_address)
        init_prior_msg.full_index = int(self.modem_addresses[local_chr][1])
        init_prior_msg.initial_position = initial_position
        init_prior_msg.initial_orientation = initial_orientation
        init_prior_msg.initial_sigmas = initial_sigmas
        # Stage the initial prior factor message for sending
        self.staged_init_prior = init_prior_msg
        self.inbound_init_priors[local_chr] = {
            "key": local_symbol,
            "position": initial_position,
            "orientation": initial_orientation,
            "sigmas": initial_sigmas
        }
        # Log that we've sent the message
        rospy.loginfo("[%s] Published Initial Prior Factor" % (rospy.Time.now()))
        return

    def build_partial_graph(self):
        """This function builds the partial graph from the cycle graph data
        - It checks that the graph is connected and notes any discrepancies (does not currently patch them, just fails)
        - It check that the graph will fit in the PartialGraph.msg
        - It builds the partial graph message from the cycle graph data
        Returns:
            PartialGraph: The built partial graph message
        """
        partial_graph = PartialGraph()
        # Check if the partial graph can be sent in the msg and is connected:
        if self.num_agents == 1 and self.num_landmarks == 0:
            rospy.loginfo("[%s] No partial graph data to send" % rospy.Time.now())
            return partial_graph
        else:
            # Check that the graph is the right size, is feasible and is connected:
            graph_check = check_partial_graph_msg(self.partial_graph_data)
            if not graph_check:
                rospy.logerror("[%s] Partial graph data check failed" % rospy.Time.now())
            else:
                rospy.loginfo("[%s] Partial graph data check passed" % rospy.Time.now())

                # Read the partial graph data into the cycle graph data
                self.cycle_graph_data = self.partial_graph_data.copy()
                # Set the local address and full index
                partial_graph.local_addr = int(self.local_address)
                partial_graph.full_index = self.modem_addresses[chr(ord("A") + self.local_address)][1]
                partial_graph.num_poses = len([key for key in self.partial_graph_data.keys() if key.startswith("BTWN")])

                # Iterate through the cycle graph data and populate the partial graph
                num_btwn = 0
                num_rng_from_us = 0
                num_rng_to_us = 0
                # There are  maximum of 6 BTWN entries and 6 RNG entries, but 12 total entries in cycle_graph_data
                for i, (graph_id, data) in enumerate(self.partial_graph_data.items()):
                    if graph_id.startswith("BTWN") and num_btwn < 6:
                        # If this is the first BTWN entry, set the initial position and orientation to the GPS data
                        if num_btwn == 0:
                            # Compose the relative pose with the failed cycle relative pose
                            if self.failed_cycle_relative_pose["key1"] is not None:
                                corr_pos, corr_ori, corr_sig = correct_first_relative_pose_for_failed_cycle(
                                    data, self.failed_cycle_relative_pose)
                                position, orientation, sigmas = encode_partial_graph_pose_as_int(
                                    corr_pos, corr_ori, corr_sig)
                            else:
                                position, orientation, sigmas = encode_partial_graph_pose_as_int(
                                    data["position"], data["orientation"], data["sigmas"])
                        else:
                            # Extract the relative position and orientation data
                            position, orientation, sigmas = encode_partial_graph_pose_as_int(
                                data["position"], data["orientation"], data["sigmas"])
                        setattr(partial_graph, f'relative_pos_{num_btwn}', position)
                        setattr(partial_graph, f'relative_rot_{num_btwn}', orientation)
                        setattr(partial_graph, f'unique_sigmas_{num_btwn}', sigmas)
                        num_btwn += 1
                    elif graph_id.startswith("RNG") and data.get("range") is not None and num_rng_from_us < 4:
                        # Get the local index and remote address from the graph data
                        local_index = int(data["key1"][1:]) - partial_graph.full_index  # Extract the index from the key:
                        remote_addr = int(self.modem_addresses[data["key2"]]) # note this may be a landmark of form "L0"
                        meas_range = encode_partial_graph_range_as_int(data["range"])
                        # Set the range data in the partial graph
                        setattr(partial_graph, f'local_index_{num_rng_from_us}', local_index)
                        setattr(partial_graph, f'remote_addr_{num_rng_from_us}', remote_addr)
                        setattr(partial_graph, f'meas_range_{num_rng_from_us}', meas_range)
                        num_rng_from_us += 1
                    elif graph_id.startswith("RNG") and data.get("range") is None and num_rng_to_us < 2:
                        local_index = int(data["key1"][1:]) - partial_graph.full_index  # Extract the index from the key:
                        remote_addr = int(self.modem_addresses[data["key2"][0]]) # this can only be an agent
                        remote_index = int(data["key2"][1:])
                        # Set the range data in the partial graph
                        setattr(partial_graph, f'local_index_{num_rng_to_us + 4}', local_index)
                        setattr(partial_graph, f'remote_addr_{num_rng_to_us + 4}', remote_addr)
                        setattr(partial_graph, f'remote_index_{num_rng_to_us + 4}', remote_index)
                        num_rng_to_us += 1
                    else:
                        rospy.logerr("[%s] Invalid graph ID: %s" % (rospy.Time.now(), graph_id))
                        continue
        return partial_graph

if __name__ == "__main__":

    try:
        cycle_mgr = CycleManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[%s] Comms Cycle Mgr Interrupted" % rospy.Time.now())
    except Exception as e:
        rospy.logerr("[%s] Comms Cycle Mgr Error: %s" % (rospy.Time.now(), e))
    finally:
        rospy.loginfo("[%s] Comms Cycle Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Comms Cycle Mgr Exiting")
        exit(0)