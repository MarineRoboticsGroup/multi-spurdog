#!/usr/bin/env python3
import rospy
import numpy as np
from datetime import datetime
import scipy.spatial.transform as spt
from ros_acomms_msgs.msg import(
    TdmaStatus
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)
from spurdog_acomms.msg import(
    InitPrior, PartialGraph, PoseWithAssoc, RangeWithAssoc, CycleGraph
)
from spurdog_acomms.srv import(
    PreintegrateIMU, PreintegrateIMUResponse
)
from std_msgs.msg import Header, String, Time, Float32
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped

from spurdog_acomms_utils.coding_utils import (
    encode_init_prior_data_as_int,
    decode_init_prior_data_from_int,
    encode_partial_graph_data_as_int,
    decode_partial_graph_data_from_int
)

class CycleManager:
    """This is a node to run the comms cycle for the vehicle."""
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.local_address = rospy.get_param("~modem_address", 0)
        self.num_agents = rospy.get_param("~num_agents", 1)
        self.num_landmarks = rospy.get_param("~num_landmarks", 0)
        self.landmarks = rospy.get_param("~landmarks", {}) # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = rospy.get_param("~sound_speed", 1500)
        self.sigma_range = rospy.get_param("~sigma_range", 1)
        # self.sigma_depth = rospy.get_param("~sigma_depth", 1)
        # self.sigma_roll = rospy.get_param("~sigma_roll", 0.1)
        # self.sigma_pitch = rospy.get_param("~sigma_pitch", 0.1)
        # Variables for addressing
        self.modem_addresses = {}
        self.cycle_target_mapping = {}
        # Variables for tdma and cycle execution
        self.tdma_status = TdmaStatus()
        self.tdma_cycle_sequence = 0
        self.msg_preload = 10 # seconds to allow for encoding the message before the tdma window opens
        # Variables for acomms event topic
        self.ping_sent_time = None
        self.ping_method = "ping with payload"
        self.ping_timeout = 5 # seconds
        self.ping_slot_open = False
        self.last_tj = None # last time we sent a ping
        self.loaded_msg = False # last message we sent to the modem
        # Variables for compressing data for codec
        self.codec_scale_factors = {
            "init_prior": {
                "x": 10.0,
                "y": 10.0,
                "z": 100.0,
                "qx": 32767,
                "qy": 32767,
                "qz": 32767,
                "qw": 32767,
                "sigma_x": 300,
                "sigma_y": 300,
                "sigma_z": 300,
                "sigma_roll": 5000,
                "sigma_pitch": 5000,
                "sigma_yaw": 5000,
            },
            "partial_graph": {
                "x": 100.0,
                "y": 100.0,
                "z": 100.0,
                "qx": 127,
                "qy": 127,
                "qz": 127,
                "qw": 127,
                "sigma_x": 10,
                "sigma_y": 10,
                "sigma_z": 10,
                "sigma_roll": 10,
                "sigma_pitch": 10,
                "sigma_yaw": 10,
                "range": 100,
            }
        }
        # Variables for external sensors
        self.imu_relative_poses = {}
        self.gps_fix = [[1,2,3],[0,0,0,1],[np.eye(6)]] # [position, orientation, covariance]
        # Variables for message handling
        self.staged_init_prior = None
        self.staged_partial_graph = None
        self.partial_graph_data = {}
        self.cycle_graph_data = {}
        self.inbound_init_priors = {}
        self.inbound_partial_graphs = {}
        self.pose_time_lookup = {}
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
        rospy.wait_for_service("preintegrate_imu")
        self.ping_client = rospy.ServiceProxy("modem/ping_modem", PingModem)
        self.preintegrate_imu = rospy.ServiceProxy("preintegrate_imu", PreintegrateIMU)
        rospy.loginfo("[%s] Services ready, initializing topics" % rospy.Time.now())

        self.tdma_from_modem = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_status, queue_size=1)
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        # Establish the message subs and pubs
        self.init_prior_sub = rospy.Subscriber("modem/from_acomms/init_prior", InitPrior, self.on_init_prior)
        self.partial_graph_sub = rospy.Subscriber("modem/from_acomms/partial_graph", PartialGraph, self.on_partial_graph)
        self.init_prior_pub = rospy.Publisher("modem/to_acomms/init_prior", InitPrior, queue_size=1)
        self.partial_graph_pub = rospy.Publisher("modem/to_acomms/partial_graph", PartialGraph, queue_size=1)
        # Initialize Subscribers for handling external sensors
        self.gps = rospy.Subscriber("gps", PoseWithCovarianceStamped, self.on_gps)
        #self.preintegrated_imu = rospy.Subscriber("preintegrated_imu", PoseWithCovarianceStamped, self.on_preintegrated_imu)
        # Initialize the pubs/subs for create relative pose measurements from the estimator
        #self.acomms_event = rospy.Publisher("acomms_event", Time, queue_size=1)
        self.cycle_graph_pub = rospy.Publisher("cycle_graph", CycleGraph, queue_size=1)
        # Initialize the modem addresses and cycle targets
        rospy.loginfo("[%s] Topics ready, initializing comms cycle" % rospy.Time.now())
        self.setup_addresses()
        self.setup_cycle_targets()
        self.build_init_prior()
        # Start the cycle
        rospy.loginfo("[%s] Comms Cycle Running" % rospy.Time.now())

    # Setup Functions: #TODO: There's an error in here (str indices not integers)
    def setup_addresses(self):
        """This function sets up the number of agents and landmarks
        """
        # Build the modem address dict, assigning a unique integer to each agent and landmark
        if self.num_agents == 1 and self.num_landmarks == 0:
            rospy.logerr("[%s] No addresses to ping" % rospy.Time.now())
        else:
            # Generate the agent addresses
            for i in range(self.num_agents):
                letter = chr(ord('A') + i)
                if i == self.local_address:
                    # Assigns an additional integer for tracking our current pose index
                    self.modem_addresses[letter] = [i, 0]
                else:
                    self.modem_addresses[letter] = [i]
            # Generate the landmark addresses
            for j in range(self.num_landmarks):
                address = j + i + 1
                #rospy.loginfo("[%s] Adding Landmark with L%d" % (rospy.Time.now(), j))
                self.modem_addresses["L%d" % j] = [address,0]
                # Add priors for the landmarks
                self.inbound_init_priors["L%d" % j] = {
                    "key": "L%d" % j,
                    "position": np.array(self.landmarks["L%d" % j]),
                    "sigmas": np.array([1.7, 1.7, 0.1])
                }
        rospy.loginfo("[%s] Modem Addresses: %s" % (rospy.Time.now(), self.modem_addresses))
        return

    def setup_cycle_targets(self):
        """This function sets up the cycle targets for the agents and landmarks
        so that we can use the tdma slot to select the target agent and landmark for that slot in the cycle"""

        # Set up the expected number of slots and the active slots:
        if self.num_agents == 1:
            expected_slots = 2
            active_slots = [0, 1]  # Only one agent, so both slots are active
            self.cycle_target_mapping = {i: [[], []] for i in range(expected_slots)}
            if self.num_landmarks == 1:
                self.cycle_target_mapping[0] = [self.modem_addresses["L0"][0], self.modem_addresses["L0"][0]]
                self.cycle_target_mapping[1] = [self.modem_addresses["L0"][0], self.modem_addresses["L0"][0]]
            elif self.num_landmarks == 2:
                self.cycle_target_mapping[0] = [self.modem_addresses["L0"][0], self.modem_addresses["L1"][0]]
                self.cycle_target_mapping[1] = [self.modem_addresses["L0"][0], self.modem_addresses["L1"][0]]
        elif self.num_agents == 2:
            expected_slots = 4
            active_slots = [self.local_address, self.local_address + 2]
            # Find the other agent address
            other_agent = 1 if self.local_address == 0 else 0
            # Create a four slot dict of empty entries
            self.cycle_target_mapping = {i: [[], []] for i in range(expected_slots)}
            # Assign the active slots to the other agent and the landmarks
            if self.num_landmarks == 0:
                self.cycle_target_mapping[active_slots[0]] = [other_agent, []]
                self.cycle_target_mapping[active_slots[1]] = [other_agent, []]
            elif self.num_landmarks == 1:
                self.cycle_target_mapping[active_slots[0]] = [other_agent, self.modem_addresses["L0"][0]]
                self.cycle_target_mapping[active_slots[1]] = [other_agent, self.modem_addresses["L0"][0]]
            elif self.num_landmarks == 2:
                self.cycle_target_mapping[active_slots[0]] = [other_agent, self.modem_addresses["L0"][0]]
                self.cycle_target_mapping[active_slots[1]] = [other_agent, self.modem_addresses["L1"][0]]
            else:
                rospy.logerr("[%s] Invalid number of landmarks: %d" % (rospy.Time.now(), self.num_landmarks))
        elif self.num_agents == 3:
            expected_slots = 6
            active_slots = [self.local_address, self.local_address + 3]
            # Find the other agent addresses
            other_agents = [i for i in range(self.num_agents) if i != self.local_address]
            # Create a six slot dict of empty entries
            self.cycle_target_mapping = {i: [[], []] for i in range(expected_slots)}
            # Assign the active slots to the other agents and the landmarks
            if self.num_landmarks == 0:
                self.cycle_target_mapping[active_slots[0]] = [other_agents[0], []]
                self.cycle_target_mapping[active_slots[1]] = [other_agents[1], []]
            elif self.num_landmarks == 1:
                self.cycle_target_mapping[active_slots[0]] = [other_agents[0], self.modem_addresses["L0"][0]]
                self.cycle_target_mapping[active_slots[1]] = [other_agents[1], self.modem_addresses["L0"][0]]
            elif self.num_landmarks == 2:
                self.cycle_target_mapping[active_slots[0]] = [other_agents[0], self.modem_addresses["L0"][0]]
                self.cycle_target_mapping[active_slots[1]] = [other_agents[1], self.modem_addresses["L1"][0]]
            else:
                rospy.logerr("[%s] Invalid number of landmarks: %d" % (rospy.Time.now(), self.num_landmarks))
        else:
            rospy.logerr("[%s] Invalid number of agents: %d" % (rospy.Time.now(), self.num_agents))
            return
        rospy.loginfo("[%s] Active Slots: %s" % (rospy.Time.now(), active_slots))
        rospy.loginfo("[%s] Cycle Target Mapping: %s" % (rospy.Time.now(), self.cycle_target_mapping))
        return

        #TDMA Cycle Tracking:

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
        msg_id = msg.header.seq
        skew_buffer = 1 # seconds on either side to account for any skew in the modem clock
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
                    self.init_prior_pub.publish(self.staged_init_prior)
                    self.loaded_msg = True
                else:
                    rospy.loginfo("[%s] Publishing Partial Graph" % rospy.Time.now())
                    self.partial_graph_pub.publish(self.staged_partial_graph)
                    self.loaded_msg = True
            # If we are active, we need to execute the ping cycle
            elif we_are_active == True:
                if elapsed_time_in_slot < 1:
                    rospy.loginfo("[%s] TDMA Active Slot Started, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
                    self.loaded_msg = False
                elif elapsed_time_in_slot > skew_buffer and remaining_sec_in_slot > skew_buffer:
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
        # self.tdma_status = msg
        self.tdma_status.current_slot = current_slot
        self.tdma_status.we_are_active = we_are_active
        self.tdma_status.remaining_slot_seconds = remaining_sec_in_slot
        self.tdma_status.remaining_active_seconds = remaining_active_sec
        self.tdma_status.time_to_next_active = time_to_next_active
        self.tdma_status.slot_duration_seconds = slot_duration
        return

    def on_tdma_cycle_reset(self):
        """This function is called when the TDMA slot returns to 0
        """
        # Process the partial graphs we've recieved into a cycle graph message to send to the estimator
        if len(self.inbound_partial_graphs.keys()) >= self.num_agents:
            # We should have synched this and sent it to the estimator already, clear the cycle data
            self.cycle_graph_data.clear()
            # clear the false cycle relative pose data (needs to be reset for the next cycle)
            self.failed_cycle_relative_pose = {
                "key1": None,
                "key2": None,
                "position": None,
                "orientation": None,
                "sigmas": None
            }
        elif len(self.inbound_partial_graphs.keys()) == 1:
            # If we only have one partial graph (ours), we should scrub this cycle and store its pose delta
            self.integrate_across_poses(chr(ord("A") + self.local_address) + "0", chr(ord("A") + self.local_address) + str(self.modem_addresses[chr(ord("A") + self.local_address)][1]))
            rospy.logwarn("[%s] Only one partial graph received, scrubbing cycle" % rospy.Time.now())
            self.cycle_graph_data.clear()
        else: # if we recieved 2 of 3 expected graphs
            rospy.loginfo
            rospy.logwarn("[%s] Not enough partial graphs recieved (%d), attempting sync anyway" % (rospy.Time.now(), len(self.inbound_partial_graphs.keys())))
            # This tries to synchronize what we have
            self.synchronize_partial_graphs()
            self.build_cycle_graph()
            self.cycle_graph_data.clear()

        # Process the previous cycle's data into a partial graph message and stage it for the next cycle
        self.staged_partial_graph = self.build_partial_graph_from_local_data()
        #TODO: This may return an empty message if there are unconnected poses, if there is only one modem in the network, or if the graph can't fit in the message.
        # Unconnected ranges are removed within the checking function, so they will not affect this.
        self.partial_graph_data.clear()

        # If we are still trying to initialize, reset the tdma_cycle sequence so we keep trying to initialize
        if len(self.inbound_init_priors.keys()) < (self.num_agents + self.num_landmarks):
            self.tdma_cycle_sequence += 1
            #self.tdma_cycle_sequence = 0
            # Get the first and last pose keys in the self.staged_partial_graph
            first_key = chr(ord("A") + self.local_address) + "0"
            last_key = chr(ord("A") + self.local_address) + str(self.staged_partial_graph.full_index + self.staged_partial_graph.num_poses)
            # Integrate across the poses to get the relative pose from first_key to last_key
            # position, orientation, sigmas = self.integrate_across_poses(first_key, last_key)
            # self.failed_cycle_relative_pose["key1"] = first_key
            # self.failed_cycle_relative_pose["key2"] = last_key
            # self.failed_cycle_relative_pose["position"] = position
            # self.failed_cycle_relative_pose["orientation"] = orientation
            # self.failed_cycle_relative_pose["sigmas"] = sigmas
            # Reset the local pose index to 0 for the next cycle (preserves the partial graph format)
            self.modem_addresses[chr(ord("A") + self.local_address)][1] = 0
            # Rebuild the initial prior data for the next cycle
            self.build_init_prior()
            rospy.logwarn("[%s] TDMA Cycle Reset, waiting for init prior data" % rospy.Time.now())
            return
        else:
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
    def on_nmea_from_modem(self, msg: String):
        """This function receives NMEA messages from the modem
        """
        # Get the NMEA data type and the data
        nmea_msg = msg.data.split('*')[0]#remove checksum
        #nmea_msg = nmea_msg[0]
        nmea_string = nmea_msg.split(",") # split into fields
        nmea_type = nmea_string[0] # Get the NMEA string type
        data = nmea_string[1:] # Get the NMEA data in list form

        # Process the NMEA data by field
        if nmea_type == "$CACMD": # Modem-to-host acknowledgement of a ping command $CACMD,PNG,0,1,1,0,3*22
            if data[0] == "PNG" and data[1] == str(self.local_address):
                self.ping_sent_time = rospy.Time.now()
                rospy.loginfo("[%s] Sent Ping to %s" % (self.ping_sent_time, data[2]))
            else:
                rospy.logerr("[%s] Received $CACMD with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMA": # Modem-to-host acknowledgement of a ping recieved $CACMA,2025-03-17T17:16:29.639994,PNG,0,1,56.7,6.62,3,0,0*74
            acomms_event_time = rospy.Time.from_sec(datetime.strptime(data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp())
            if data[1] == "PNG" and data[3] == str(self.local_address):
                self.request_preintegration(acomms_event_time) # Request a relative pose measurement
                self.modem_addresses[chr(ord("A") + self.local_address)][1] += 1
                rospy.loginfo("[%s] Received Ping from %s" % (acomms_event_time, data[3]))
            elif data[1] == "PNG":
                rospy.loginfo("[%s] Overheard Ping from %s to %s" % (acomms_event_time, data[2], data[3]))
            else:
                rospy.logerr("[%s] Received $CACMA with unexpected data: %s" % (rospy.Time.now(), data))

        # TODO: Fix this once you observe one (publishing the time actually happens in the ping service)
        elif nmea_type == "$CACMR": # Modem-to-host acknowledgement of a ping response $CACMR,PNR,SRC,DEST,????
            acomms_event_time = rospy.Time.from_sec(datetime.strptime(data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp())
            if data[1] == "PNR" and data[3] == str(self.local_address):
                #self.acomms_event.publish(acomms_event_time)
                #self.modem_addresses[chr(ord("A") + self.local_address)][1] += 1
                rospy.loginfo("[%s] Received Ping Response from %s" % (acomms_event_time, data[3]))
            elif data[1] == "PNR":
                rospy.loginfo("[%s] Overheard Ping Response from %s to %s" % (acomms_event_time, data[2], data[3]))
            else:
                rospy.logerr("[%s] Received $CACMR with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CARFP" and data[5] == "-1": # Modem-to-host acknowledgement of a minipacket ping payload
            time, src, dest, payload = self.extract_ping_data_from_carfp(data)
            if not time or not src or not dest or not payload:
                rospy.logerr("[%s] CARFP message is missing required fields" % rospy.Time.now())
                return
            elif dest == str(self.local_address):
                rospy.loginfo("[%s] Received Ping from %s with payload %s" % (time, src, dest, payload))
            elif dest != str(self.local_address):
                rospy.logerr("[%s] Overheard Ping-related $CARFP from %s to %s with paylaod %s" % (time, src, dest, payload))
            else:
                rospy.logerr("[%s] Received $CARFP with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CAREV" and data[1] == "AUV" and self.ping_method == None: # Modem-to-host $CAREV message to determine the firmware version
            firmware_version = data[2].split(".")
            if firmware_version[0] == "3":
                # New deckbox firmware
                self.ping_method = "ping with payload"
            else:
                # Old deckbox firmware
                self.ping_method = "no payload"
        else:
            return
        return

    def extract_ping_data_from_carfp(self, data):
        """This function extracts the ping payload data from a ping-related $CARFP NMEA message
        Args:
            data (list): The NMEA data fields [HHHH, SRC, DEST, PAYLOAD]
        Returns:
            tuple: (time, src, dest, hex_payload)
        """
        time = rospy.Time.from_sec(datetime.strptime(data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp())
        src = int(data[2])
        dest = int(data[3])
        hex_data = data[9].split(";")[-1]  # Splits the data into frames, then takes the last frame
        hex_payload = bytearray.fromhex(hex_data)
        payload = hex_payload.decode("utf-8") if hex_payload else ""
        return time, src, dest, payload

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
                #self.acomms_event.publish(timestamp)
                self.request_preintegration(timestamp) # Request a relative pose measurement
                self.modem_addresses[chr(ord("A") + self.local_address)][1] += 1
                # Get the key association from the src and dest address
                src_chr = chr(ord("A") + src)
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
    def request_preintegration(self, tj):
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
                rospy.loginfo(f"Received preintegrated pose betwwene {ti} and {tj}")
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
                # Store the relative pose data in the imu_relative_poses dict
                self.imu_relative_poses[tj] = {
                    "key1": key1,
                    "key2": key2,
                    "position": position,
                    "orientation": orientation,
                    "sigmas": sigmas
                }
                # Add to the cycle graph data
                graph_id = "BTWN_%s_%s" % (key1, key2)
                self.partial_graph_data[graph_id] = {
                    "key1": key1,
                    "key2": key2,
                    "position": np.array([position.x, position.y, position.z]),
                    "orientation": np.array([orientation.x, orientation.y, orientation.z, orientation.w]),
                    "sigmas": sigmas
                }
                # Remove any imu relative poses that are older than the current pose time -180sec
                self.imu_relative_poses = {time: data for time, data in self.imu_relative_poses.items() if time >= (tj - rospy.Duration(180))}
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return

    #TODO: Review the math for these two functions:
    def integrate_across_poses(self, first_key, second_key):
        """This function integrates the relative poses across the imu_relative_poses dict
        Args:
            first_key (str): The key for the first pose ("A0")
            second_key (str): The key for the second pose ("A5")
        Returns:
            The delta pose and covariance from A0 to A5
        """
        sorted_times = sorted(self.imu_relative_poses.keys())
        # Get the pose times
        time_of_first_key = self.pose_time_lookup[first_key]
        time_of_second_key = self.pose_time_lookup[second_key]
        # Integrate all the relative poses between time of first_key and time of second_key
        sorted_times = [time for time in sorted_times if time_of_first_key <= time <= time_of_second_key]
        delta_position = np.zeros(3)
        delta_orientation = spt.Rotation.identity()  # Identity rotation (quaternion)
        delta_covariance = np.zeros((6, 6))  # Full 6x6 covariance matrix
        # Iterate through the sorted times and accumulate the relative poses
        for time in sorted_times:
            data = self.imu_relative_poses[time]

            # Extract relative motion components
            rel_position = np.array(data["position"])  # Relative translation
            rel_orientation = spt.Rotation.from_quat(data["orientation"])  # Relative quaternion
            rel_covariance = np.diag(data["sigmas"]**2)  # Convert std deviations to diagonal covariance matrix

            # Transform position incrementally using the current rotation
            delta_position += delta_orientation.apply(rel_position)

            # Apply quaternion multiplication to integrate orientation
            delta_orientation = delta_orientation * rel_orientation

            # Propagate covariance (assuming independent errors)
            delta_covariance += rel_covariance @ rel_covariance.T  # Accumulate variances

        # Convert covariance back to standard deviations
        delta_sigmas = np.sqrt(np.diag(delta_covariance))

        return delta_position, delta_orientation.as_quat(), delta_sigmas

    def compose_relative_pose_with_failed_cycle(self, position, orientation, sigmas):
        """This function composes a relative pose from the position, orientation and sigmas
        Args:
            position (np.array): The position as a 3D vector
            orientation (np.array): The orientation as a quaternion
            sigmas (np.array): The sigmas as a 6D vector
        Returns:
            A dictionary with the composed relative pose data
        """
        # Compose the pose represented by the position, orientation, sigmas with the failed cycle data
        if self.failed_cycle_relative_pose["key1"] is not None and self.failed_cycle_relative_pose["key2"] is not None:
            existing_orientation = spt.Rotation.from_quat(self.failed_cycle_relative_pose["orientation"])
            existing_covariance = np.diag(self.failed_cycle_relative_pose["sigmas"]**2)
            new_position = self.failed_cycle_relative_pose["position"] + existing_orientation.apply(position)
            new_orientation = existing_orientation * spt.Rotation.from_quat(orientation)
            new_covariance = existing_covariance + (np.diag(sigmas)**2)
            new_sigmas = np.sqrt(np.diag(new_covariance))
            position = new_position
            orientation = new_orientation.as_quat()
            sigmas = new_sigmas
        else:            # If there is no failed cycle data, just return the original values
            pass
        return {
            "position": position,
            "orientation": orientation,
            "sigmas": sigmas
        }

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

    # Pre-encoding and decoding functions:
    # def encode_init_prior_data_as_int(self, initial_position, initial_orientation, initial_sigmas):
    #     """This function encodes the initial prior factor data into a message
    #     Args:
    #         initial_position (np.array): The initial position
    #         initial_orientation (np.array): The initial orientation
    #         initial_sigmas (np.array): The initial sigmas
    #     Returns:
    #         PosePriorFactor: The encoded initial prior factor message
    #     """
    #     # Position is a int16[3], multiply by scale, then convert to int16
    #     x = int(initial_position[0] * self.codec_scale_factors["init_prior"]["x"])
    #     y = int(initial_position[1] * self.codec_scale_factors["init_prior"]["y"])
    #     z = int(initial_position[2] * self.codec_scale_factors["init_prior"]["z"])
    #     # verify all fields are within int16 range
    #     if not (np.all(np.abs([x, y, z]) <= 32767)):
    #         rospy.logerr("[%s] Initial position values out of range!" % rospy.Time.now())
    #         return None
    #     else:
    #         initial_position = np.array([x, y, z], dtype=np.int16)

    #     # Orientation is a int16[4] (quaternion)
    #     qx = int(initial_orientation[0] * self.codec_scale_factors["init_prior"]["qx"])
    #     qy = int(initial_orientation[1] * self.codec_scale_factors["init_prior"]["qy"])
    #     qz = int(initial_orientation[2] * self.codec_scale_factors["init_prior"]["qz"])
    #     qw = int(initial_orientation[3] * self.codec_scale_factors["init_prior"]["qw"])
    #     # verify all fields are within int16 range
    #     if not (np.all(np.abs([qx, qy, qz, qw]) <= 32767)):
    #         rospy.logerr("[%s] Initial orientation values out of range!" % rospy.Time.now())
    #         return None
    #     else:
    #         initial_orientation = np.array([qx, qy, qz, qw], dtype=np.int16)

    #     # Sigmas is a int16[6], multiply by scale, then convert to int16
    #     sx = int(initial_sigmas[0] * self.codec_scale_factors["init_prior"]["sigma_x"])
    #     sy = int(initial_sigmas[1] * self.codec_scale_factors["init_prior"]["sigma_y"])
    #     sz = int(initial_sigmas[2] * self.codec_scale_factors["init_prior"]["sigma_z"])
    #     sroll = int(initial_sigmas[3] * self.codec_scale_factors["init_prior"]["sigma_roll"])
    #     spitch = int(initial_sigmas[4] * self.codec_scale_factors["init_prior"]["sigma_pitch"])
    #     syaw = int(initial_sigmas[5] * self.codec_scale_factors["init_prior"]["sigma_yaw"])
    #     # verify all fields are within int16 range
    #     if not (np.all(np.abs([sx, sy, sz, sroll, spitch, syaw]) <= 32767)):
    #         rospy.logerr("[%s] Initial sigmas values out of range!" % rospy.Time.now())
    #         return None
    #     else:
    #         initial_sigmas = np.array([sx, sy, sz, sroll, spitch, syaw], dtype=np.int16)
    #     return initial_position, initial_orientation, initial_sigmas

    # def decode_init_prior_data_from_int(self, initial_position, initial_orientation, initial_sigmas):
    #     # Apply the reverse of the encoding process to decode the data
    #     # Position is a int16[3], divide by scale to get original value
    #     initial_position = np.array([
    #         initial_position[0] / self.codec_scale_factors["init_prior"]["x"],
    #         initial_position[1] / self.codec_scale_factors["init_prior"]["y"],
    #         initial_position[2] / self.codec_scale_factors["init_prior"]["z"]
    #     ])
    #     # Orientation is a int16[4] (quaternion), divide by scale to get original value
    #     initial_orientation = np.array([
    #         initial_orientation[0] / self.codec_scale_factors["init_prior"]["qx"],
    #         initial_orientation[1] / self.codec_scale_factors["init_prior"]["qy"],
    #         initial_orientation[2] / self.codec_scale_factors["init_prior"]["qz"],
    #         initial_orientation[3] / self.codec_scale_factors["init_prior"]["qw"]
    #     ])
    #     # Sigmas is a int16[6], divide by scale to get original value
    #     initial_sigmas = np.array([
    #         initial_sigmas[0] / self.codec_scale_factors["init_prior"]["sigma_x"],
    #         initial_sigmas[1] / self.codec_scale_factors["init_prior"]["sigma_y"],
    #         initial_sigmas[2] / self.codec_scale_factors["init_prior"]["sigma_z"],
    #         initial_sigmas[3] / self.codec_scale_factors["init_prior"]["sigma_roll"],
    #         initial_sigmas[4] / self.codec_scale_factors["init_prior"]["sigma_pitch"],
    #         initial_sigmas[5] / self.codec_scale_factors["init_prior"]["sigma_yaw"]
    #     ])
    #     return initial_position, initial_orientation, initial_sigmas

    # def encode_partial_graph_data_as_int(self, id, position, orientation, sigmas):
    #     """This function encodes the partial graph data into a message
    #     Args:
    #         position (np.array): The relative position
    #         orientation (np.array): The relative orientation
    #         sigmas (np.array): The relative sigmas
    #     Returns:
    #         PartialGraph: The encoded partial graph message
    #     """
    #     # Position is a int8[3], multiply by scale, then convert to int8
    #     x = int(position[0] * self.codec_scale_factors["partial_graph"]["x"])
    #     y = int(position[1] * self.codec_scale_factors["partial_graph"]["y"])
    #     z = int(position[2] * self.codec_scale_factors["partial_graph"]["z"])
    #     # verify all fields are within int8 range
    #     if not (np.all(np.abs([x, y, z]) <= 32767)):
    #         rospy.logerr("[%s] Relative position values out of range!" % rospy.Time.now())
    #         return None
    #     else:
    #         position = list([x, y, z])

    #     # Orientation is a int8[4] (quaternion), multiply by scale, then convert to int8
    #     qx = int(orientation[0] * self.codec_scale_factors["partial_graph"]["qx"])
    #     qy = int(orientation[1] * self.codec_scale_factors["partial_graph"]["qy"])
    #     qz = int(orientation[2] * self.codec_scale_factors["partial_graph"]["qz"])
    #     qw = int(orientation[3] * self.codec_scale_factors["partial_graph"]["qw"])
    #     # verify all fields are within int8 range
    #     if not (np.all(np.abs([qx, qy, qz, qw]) <= 127)):
    #         rospy.logerr("[%s] Relative orientation values out of range!" % rospy.Time.now())
    #         return None
    #     else:
    #         orientation = list([qx, qy, qz, qw])

    #     # Sigmas is a int8[6], multiply by scale, then convert to int8
    #     sx = int(sigmas[0] * self.codec_scale_factors["partial_graph"]["sigma_x"])
    #     sy = int(sigmas[1] * self.codec_scale_factors["partial_graph"]["sigma_y"])
    #     sz = int(sigmas[2] * self.codec_scale_factors["partial_graph"]["sigma_z"])
    #     sroll = int(sigmas[3] * self.codec_scale_factors["partial_graph"]["sigma_roll"])
    #     spitch = int(sigmas[4] * self.codec_scale_factors["partial_graph"]["sigma_pitch"])
    #     syaw = int(sigmas[5] * self.codec_scale_factors["partial_graph"]["sigma_yaw"])
    #     # verify all fields are within int8 range
    #     if not (np.all(np.abs([sx, sy, sz, sroll, spitch, syaw]) <= 255)):
    #         rospy.logerr("[%s] Relative sigmas values out of range!" % rospy.Time.now())
    #         return None
    #     else:
    #         sigmas = list([sx, sy, sz, sroll, spitch, syaw])
    #     return position, orientation, sigmas

    # def decode_partial_graph_data_from_int(self, position, orientation, sigmas):
    #     """This function decodes the partial graph data from a message
    #     Args:
    #         position (list): The relative position
    #         orientation (list): The relative orientation
    #         sigmas (list): The relative sigmas
    #     Returns:
    #         tuple: (position, orientation, sigmas)
    #     """
    #     # Decode the data by applying the reverse of the encoding process
    #     # Position is a int16[3], divide by scale to get original value
    #     position = np.array([
    #         position[0] / self.codec_scale_factors["partial_graph"]["x"],
    #         position[1] / self.codec_scale_factors["partial_graph"]["y"],
    #         position[2] / self.codec_scale_factors["partial_graph"]["z"]
    #     ])

    #     # Orientation is a int8[4], divide by scale to get original value
    #     orientation = np.array([
    #         orientation[0] / self.codec_scale_factors["partial_graph"]["qx"],
    #         orientation[1] / self.codec_scale_factors["partial_graph"]["qy"],
    #         orientation[2] / self.codec_scale_factors["partial_graph"]["qz"],
    #         orientation[3] / self.codec_scale_factors["partial_graph"]["qw"]
    #     ])

    #     # Sigmas is a int8[6], divide by scale to get original value
    #     sigmas = np.array([
    #         sigmas[0] / self.codec_scale_factors["partial_graph"]["sigma_x"],
    #         sigmas[1] / self.codec_scale_factors["partial_graph"]["sigma_y"],
    #         sigmas[2] / self.codec_scale_factors["partial_graph"]["sigma_z"],
    #         sigmas[3] / self.codec_scale_factors["partial_graph"]["sigma_roll"],
    #         sigmas[4] / self.codec_scale_factors["partial_graph"]["sigma_pitch"],
    #         sigmas[5] / self.codec_scale_factors["partial_graph"]["sigma_yaw"]
    #     ])

    #     return position, orientation, sigmas

    # Message Processing Functions
    def check_partial_graph_data_for_completeness(self):
        """ This checks the graph data to ensure that we recorded the expected number of events
        - It does not prevent sending the graph if its incomplete
        Output:
            bool: True if the graph data is complete and valid, False otherwise
            - True indicates its connected and can be sent to the estimator.
            - False indicates its either empty or not connected and should not be sent to the estimator
        """
         # Check that we have data to process
        if self.num_agents == 1 and self.num_landmarks == 0:
            return True
        elif not self.partial_graph_data:
            rospy.logerr("[%s] Cycle graph data is empty" % rospy.Time.now())
            return False
        else:
            # Using the cycle target mapping, check that we have the expected number of relative_poses, intialted_ranges, and recieved_ranges
            expected_num_initiated_ranges = sum([1 for pair in self.cycle_target_mapping if pair[0] != []]) + sum([1 for pair in self.cycle_target_mapping if pair[1] != []])
            expected_num_received_ranges = 2 if self.num_agents > 1 else 0
            expected_num_relative_poses = expected_num_initiated_ranges + expected_num_received_ranges
            # Check the number of recorded entries of each type
            num_btwn = len([key for key in self.partial_graph_data.keys() if key.startswith("BTWN")])
            num_rng_from_us = len([key for key in self.partial_graph_data.keys() if key.startswith("RNG") and self.partial_graph_data[key].get("range") is not None])
            num_rng_to_us = len([key for key in self.partial_graph_data.keys() if key.startswith("RNG") and self.partial_graph_data[key].get("range") is None])

            # Check the between factors
            if num_btwn != expected_num_relative_poses:
                rospy.logerr("[%s] Relative pose mismatch, recorded: %d, expected: %d" % (rospy.Time.now(), num_btwn, expected_num_relative_poses))
            # NOTE: This indicates one of the ranging events failed. The relative pose will be stretched to the following ranging event.
            # So its probably fine to let this go forward
            else:
                rospy.loginfo("[%s] Relative poses match expected number: %d" % (rospy.Time.now(), num_btwn))
            # Check the ranges we initiated
            if num_rng_from_us != expected_num_initiated_ranges:
                rospy.logerr("[%s] Initiated range mismatch, recorded: %d, expected: %d" % (rospy.Time.now(), num_rng_from_us, expected_num_initiated_ranges))
            # NOTE: This indicates one of the ranges we initiated failed. There will be no relative pose created or range entry created unless a reply is recieved.
            # So this is fine to let go forward
            else:
                rospy.loginfo("[%s] Initiated  ranges match expected number: %d" % (rospy.Time.now(), num_rng_from_us))
            # Check the ranges we received
            if num_rng_to_us != expected_num_received_ranges:
                rospy.logerr("[%s] Received range mismatch, recorded: %d, expected: %d" % (rospy.Time.now(), num_rng_to_us, expected_num_received_ranges))
            # NOTE: This indicates an attempt to range us was not recieved. There will be no relative pose created or range entry unless it was recieved.
            # So this is fine to let go forward.
            else:
                rospy.loginfo("[%s] Received ranges match expected number: %d" % (rospy.Time.now(), num_rng_to_us))

            # Check that the relative pose entries are sequential
            local_pose_keys = []
            for i in range(num_btwn - 1):
                key1 = "BTWN_%s_%s" % (chr(ord("A") + self.local_address), i)
                key2 = "BTWN_%s_%s" % (chr(ord("A") + self.local_address), i + 1)
                if key2 not in self.partial_graph_data:
                    rospy.logerr("[%s] Missing key2: %s" % (rospy.Time.now(), key2))
                    return False # There is no way to recover from this, but it also should not happen
                elif self.partial_graph_data[key1]["key2"] != self.partial_graph_data[key2]["key1"]:
                    rospy.logerr("[%s] Key mismatch between %s and %s" % (rospy.Time.now(), key1, key2))
                    return False # There is no way to recover from this, but it also should not happen
                else:
                    local_pose_keys.append(key1)
            # Check that the RNG entries each include a key1 that is in the BTWN entries
            for key in self.partial_graph_data.keys():
                if key.startswith("RNG"):
                    if self.partial_graph_data[key]["key1"] not in local_pose_keys:
                        rospy.logerr("[%s] RNG entry %s does not match any BTWN entry" % (rospy.Time.now(), key))
                        # remove the offending range (this is a soft patch to allow it to go forward)
                        del self.partial_graph_data[key]
                    else:
                        pass
            return True

    def check_partial_graph_data_for_comms(self):
        """This function checks the partial graph data for completeness and
        whether is fits within the PartialGraph.msg
        - It does not assume that the graph is the maximum size, just that its
        the right size for the number of agents and landmarks
        Returns:
            bool: True if the partial graph data fits the restrictions of the PartialGraph.msg
        """
        num_btwn = len([key for key in self.partial_graph_data.keys() if key.startswith("BTWN")])
        num_rng_from_us = len([key for key in self.partial_graph_data.keys() if key.startswith("RNG") and self.partial_graph_data[key].get("range") is not None])
        num_rng_to_us = len([key for key in self.partial_graph_data.keys() if key.startswith("RNG") and self.partial_graph_data[key].get("range") is None])

        # Perform supportability check
        if num_btwn < 0 or num_btwn > 6:
            rospy.logerr("[%s] Unsupported number of BTWN entries: %d" % (rospy.Time.now(), num_btwn))
            return False
        elif num_rng_from_us < 0 or num_rng_from_us > 4:
            rospy.logerr("[%s] Unsupported number of Initiated RNG entries: %d" % (rospy.Time.now(), num_rng_from_us))
            return False
        elif num_rng_to_us < 0 or num_rng_to_us > 2:
            rospy.logerr("[%s] Unsupported number of Received RNG entries: %d" % (rospy.Time.now(), num_rng_to_us))
            return False
        else:
            return True

    def process_inbound_partial_graph(self, msg:PartialGraph):
        # Unpack the fields
        local_addr = chr(msg.local_addr + ord("A"))
        full_index = msg.full_index
        num_poses = msg.num_poses
        # Generate a list of the expected poses
        expected_poses = [local_addr + str(full_index + i) for i in range(num_poses)]
        # For each relative pose, decode the data:
        for i in range(num_poses):
            position, rotation, sigmas = self.decode_partial_graph_data_from_int(
                getattr(msg, f'relative_pos_{i}'),
                getattr(msg, f'relative_rot_{i}'),
                getattr(msg, f'unique_sigmas_{i}')
            )
            # Add to the cycle graph data
            graph_id = f'BTWN_{expected_poses[i]}_{expected_poses[i+1]}'
            self.cycle_graph_data[graph_id] = {
                "key1": expected_poses[i],
                "key2": expected_poses[i+1],
                "position": position,
                "orientation": rotation,
                "sigmas": sigmas
            }
        # For each range measurement, decode the data:
            if i < 4:
                local_symbol = expected_poses[int(getattr(msg, f'local_index_{i}'))]
                remote_addr = chr(getattr(msg, f'remote_addr_{i}')+ord("A"))
                meas_range = getattr(msg, f'meas_range_{i}') / self.codec_scale_factors["partial_graph"]["range"]
                # NOTE: by reversing the order of the keys, we match the convention of the existing range entry
                # It will not overwrite the existing entry, because key1 is just a chr but the key1 on file is a symbol
                graph_id = f'RNG_{remote_addr}_{local_symbol}'
                self.cycle_graph_data[graph_id] = {
                    "key1": remote_addr,
                    "key2": local_symbol,
                    "range": meas_range
                }
            elif i < 6:
                local_symbol = expected_poses[int(getattr(msg, f'local_index_{i}'))]
                remote_addr = chr(getattr(msg, f'remote_addr_{i}')+ord("A"))
                remote_index = getattr(msg, f'remote_index_{i}')
                remote_symbol = chr(ord("A") + remote_addr) + str(remote_index)
                # NOTE: by reversing the order of the keys, we match the order of the existing range entry
                # It will not overwrite the existing entry, because key2 is a symb, but key2 on file is a chr
                graph_id = f'RNG_{remote_symbol}_{local_symbol}'
                self.cycle_graph_data[graph_id] = {
                    "key1": remote_symbol,
                    "key2": local_symbol,
                    "range": None
                }
            else:
                rospy.logerr("[%s] Invalid range measurement index: %d" % (rospy.Time.now(), i))
                continue
        return

    def synchronize_partial_graphs(self):
        """This function synchronizes the partial graphs once all expected agents have reported
        - It assumes that process_partial_graph has been called for each agent's partial graph
        - It associates the ranges between graphs, checks connectivity and adds range sigmas
        - It can be called on an insufficient number of agents, but will only process the data that is available
        and will remove any unconnected entries
        """
        # Clean up the range associations by associating the duplicate entries and removing the duplicates
        for key in self.cycle_graph_data.keys():
            if key.startswith("RNG") and len(self.cycle_graph_data[key]["key2"]) == 1:
                # Search the remaining RNG entries for a matching key1
                chr = self.cycle_graph_data[key]["key2"]
                for match in self.cycle_graph_data.keys():
                    if match.startswith("RNG") and self.cycle_graph_data[match]["key1"] == self.cycle_graph_data[key]["key1"]:
                        # Double check that the chr matches the key2 of the match
                        if self.cycle_graph_data[match]["key2"][0] == chr:
                            # If so, set the range of the key entry to the value measured in the match
                            self.cycle_graph_data[match]["range"] = self.cycle_graph_data[key]["range"]
                            # Remove the key entry from the cycle graph data
                            del self.cycle_graph_data[key]
                            break
                        else:
                            rospy.logerr("[%s] Mismatched key2 character for range entry %s" % (rospy.Time.now(), key))
                            continue
                    else:
                        rospy.logerr("[%s] No matching association found for range entry %s" % (rospy.Time.now(), key))

        # Now we need to check the connectivity of the cycle graph data
        # Build a list of the keys in each vehicle's pose chain BTWN_key1_key2 -> [key1, key2, ...]
        nodes = []
        for key in self.cycle_graph_data.keys():
            if key.startswith("BTWN"):
                key1 = self.cycle_graph_data[key]["key1"]
                key2 = self.cycle_graph_data[key]["key2"]
                if key1 not in nodes:
                    nodes.append(key1)
                if key2 not in nodes:
                    nodes.append(key2)
        # Then append the landmarks to the poses list
        if self.num_landmarks > 0:
            for i in range(self.num_landmarks):
                nodes.append("L%d" % i)

        # Check that the ranges are associated with a pose and contain a measurement
        for key in self.cycle_graph_data.keys():
            if key.startswith("RNG"):
                key1_check = True
                key2_check = True
                range_check = True
                # Check that the key1 is in the poses list
                if self.cycle_graph_data[key]["key1"] not in nodes:
                    key1_check = False
                    continue
                    # Check that the key2 is in the poses list
                if self.cycle_graph_data[key]["key2"] not in nodes:
                    key2_check = False
                    continue
                # Check that the range is not None
                if self.cycle_graph_data[key]["range"] is None:
                    range_check = False
                    continue

                # If all checks pass, add the range to the cycle graph data
                if key1_check and key2_check and range_check:
                    self.cycle_graph_data[key]["range_sigma"] = self.sigma_range
                elif (not key1_check or not key2_check) and range_check:
                    rospy.logerr("[%s] Recieved range failed to associate, removing" % (rospy.Time.now(), key))
                    del self.cycle_graph_data[key]
                elif (key1_check and key2_check) and not range_check:
                    rospy.logerr("[%s] Transmitted range failed to associate, removing" % (rospy.Time.now(), key))
                    del self.cycle_graph_data[key]
                elif not key1_check and not key2_check :
                    rospy.loginfo("[%s] Range entry %s has no associated in poses, removing" % (rospy.Time.now(), key))
                    del self.cycle_graph_data[key]
                else:
                    rospy.logerr("[%s] Range entry %s is incomplete!" % (rospy.Time.now(), key))

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
        initial_position, initial_orientation, initial_sigmas = encode_init_prior_data_as_int([0,0,0], [0,0,0,1], [1.7, 1.7, 0.1, 0.5, 0.5, 0.5])
        #initial_position, initial_orientation, initial_sigmas = encode_init_prior_data_as_int(self.gps_fix[0], self.gps_fix[1], self.gps_fix[2])
        if initial_position is None or initial_orientation is None or initial_sigmas is None:
            rospy.logerr("[%s] Failed to encode initial prior factor data!" % rospy.Time.now())
            return
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

    def build_partial_graph_from_local_data(self):
        """This function builds the partial graph from the cycle graph data
        - It checks that the graph is connected and notes any discrepancies
        - It check that the graph will fit in the PartialGraph.msg
        - It builds the partial graph message from the cycle graph data
        Returns:
            PartialGraph: The built partial graph message
        """
        partial_graph = PartialGraph()
        # Check if the partial graph is connected:
        if not self.check_partial_graph_data_for_completeness():
            rospy.logerr("[%s] Partial graph data is incomplete" % rospy.Time.now())
            return partial_graph
        elif self.num_agents == 1 and self.num_landmarks == 0:
            rospy.loginfo("[%s] No partial graph data to send" % rospy.Time.now())
            return partial_graph
        else:
            # Read the partial graph data into the cycle graph data
            self.cycle_graph_data = self.partial_graph_data.copy()
            # Check if the partial graph data fits within the PartialGraph.msg
            if not self.check_partial_graph_data_for_comms():
                rospy.logerr("[%s] Partial graph data does not fit within the PartialGraph.msg" % rospy.Time.now())
                return partial_graph
            else:
                rospy.loginfo("[%s] Partial graph data fits within the PartialGraph.msg" % rospy.Time.now())
                # Create the partial graph message
                # Initialize the partial graph message

                # Set the local address and full index
                partial_graph.local_addr = int(self.local_address)
                partial_graph.full_index = self.modem_addresses[chr(ord("A") + self.local_address)][1]
                # Set the number of poses (between 0 and 6) using the number of "BTWN" entries in the cycle graph data
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
                            relative_position, relative_orientation, relative_sigmas = self.compose_relative_pose_with_failed_cycle(data["position"], data["orientation"], data["sigmas"])
                            position, orientation, sigmas = encode_partial_graph_data_as_int(0, relative_position, relative_orientation, relative_sigmas)
                        else:
                            # Extract the relative position and orientation data
                            position, orientation, sigmas = encode_partial_graph_data_as_int(num_btwn, data["position"], data["orientation"], data["sigmas"])
                        setattr(partial_graph, f'relative_pos_{num_btwn}', position)
                        setattr(partial_graph, f'relative_rot_{num_btwn}', orientation)
                        setattr(partial_graph, f'unique_sigmas_{num_btwn}', sigmas)
                        num_btwn += 1
                    elif graph_id.startswith("RNG") and data.get("range") is not None and num_rng_from_us < 4:
                        # Get the local index and remote address from the graph data
                        local_index = int(data["key1"][1:]) - partial_graph.full_index  # Extract the index from the key:
                        remote_addr = int(self.modem_addresses[data["key2"]]) # note this may be a landmark of form "L0"
                        meas_range = int(data["range"] * self.codec_scale_factors["partial_graph"]["range"])
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
                # Log that we've "received" the partial graph
                self.inbound_partial_graphs[chr(partial_graph.local_addr + ord("A"))] = True
        return partial_graph

    def build_cycle_graph(self):
        """This function builds the cycle graph from the staged data
        Returns:
            PartialGraph: The built partial graph message
        """
        # Crete the message
        msg = CycleGraph()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # Load all the relative poses from the self.cycle_graph_data dict
        for key, data in self.cycle_graph_data.items():
            if key.startswith("BTWN"):
                position, orientation, sigmas = encode_partial_graph_data_as_int(0, data["position"], data["orientation"], data["sigmas"])
                relative_pose = PoseWithAssoc()
                relative_pose.key1 = data["key1"]
                relative_pose.key2 = data["key2"]
                relative_pose.pose.position = Point(*position)
                relative_pose.pose.orientation = Quaternion(*orientation)
                relative_pose.covariance = np.diag(sigmas**2).flatten().tolist()  # Flatten the covariance matrix to a list
                msg.relative_poses.append(relative_pose)

            elif key.startswith("RNG"):
                range_measurments = RangeWithAssoc()
                range_measurments.key1 = data["key1"]
                range_measurments.key2 = data["key2"]
                range_measurments.meas_range = data["range"]
                range_measurments.range_sigma = self.sigma_range
                msg.range_measurements.append(range_measurments)

            else:
                rospy.logerr("[%s] Invalid graph ID: %s" % (rospy.Time.now(), key))
                continue
        # Publish the cycle graph message
        self.cycle_graph_pub.publish(msg)
        rospy.loginfo("[%s] Published Cycle Graph" % rospy.Time.now())

    def on_init_prior(self, msg):
        """This function processes initial prior factor messages
        Args:
            msg (PosePriorFactor): The initial prior factor message
        """
        # Unpack the fields
        local_addr = msg.local_addr
        full_index = msg.full_index
        initial_position = msg.initial_position
        initial_orientation = msg.initial_orientation
        initial_sigmas = msg.initial_sigmas
        # Decode the initial prior factor data
        initial_position, initial_orientation, initial_sigmas = self.decode_init_prior_data_from_int(initial_position, initial_orientation, initial_sigmas)
        # Store in the inbound_init_priors dict
        self.inbound_init_priors[chr(ord("A") + local_addr)] = {
            "key": local_addr+str(full_index),
            "initial_position": initial_position,
            "initial_orientation": initial_orientation,
            "initial_sigmas": initial_sigmas
        }
        rospy.loginfo("[%s] Received Initial Prior Factor from %s" % (rospy.Time.now(),local_addr))

        #Check if the number of keys in the dict is equal to the number of agents
        if len(self.inbound_init_priors) == self.num_agents:
            # If so, we have received all initial prior factors
            rospy.loginfo("[%s] Received all Initial Prior Factors from all agents" % rospy.Time.now())
            # Add the pose priors to a cycle graph message and send to the estimator
            msg = CycleGraph()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            for key, data in self.inbound_init_priors.items():
                pose_prior = PoseWithAssoc()
                pose_prior.key1 = data["key"]
                pose_prior.key2 = data["key"]
                pose_prior.pose.position = Point(*data["initial_position"])
                pose_prior.pose.orientation = Quaternion(*data["initial_orientation"])
                pose_prior.covariance = np.diag(data["initial_sigmas"]**2).flatten().tolist()
                msg.relative_poses.append(pose_prior)
            # Publish the cycle graph message
            self.cycle_graph_pub.publish(msg)
            rospy.loginfo("[%s] Published Cycle Graph with Initial Prior Factors" % rospy.Time.now())
        else:
            rospy.loginfo("[%s] Waiting for all Initial Prior Factors from agents" % rospy.Time.now())
        return

    def on_partial_graph(self, msg):
        """This function processes partial graph messages
        Args:
            msg (PartialGraph): The partial graph message
        """
        # Unpack the fields
        local_addr = chr(msg.local_addr + ord("A"))
        full_index = msg.full_index
        num_poses = msg.num_poses
        # Log that we've recieved the partial graph
        if self.inbound_partial_graphs[local_addr] is None:
            self.inbound_partial_graphs[local_addr] = True
            self.process_inbound_partial_graph(msg)
            # Check if we have received partial graphs from the agents
            if len(self.inbound_partial_graphs.keys()) == self.num_agents:
                rospy.loginfo("[%s] Received Partial Graphs from all agents" % rospy.Time.now())
                # Synchronize the data
                self.synchronize_partial_graphs()
                # Send the graph and send to the estimator
                self.build_cycle_graph()
            else:
                rospy.loginfo("[%s] Received Partial Graph from %s" % (rospy.Time.now(), local_addr))
        else:
            rospy.logerr("[%s] Received duplicate Partial Graph from %s" % (rospy.Time.now(), local_addr))
        return

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