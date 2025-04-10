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
from std_msgs.msg import Header, String, Time, Float32
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
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
        self.sigma_depth = rospy.get_param("~sigma_depth", 1)
        self.sigma_roll = rospy.get_param("~sigma_roll", 0.1)
        self.sigma_pitch = rospy.get_param("~sigma_pitch", 0.1)
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
                "x_0": 10.0,
                "y_0": 10.0,
                "z_0": 10.0,
                "x": 10.0,
                "y": 10.0,
                "z": 10.0,
                "qx": 127,
                "qy": 127,
                "qz": 127,
                "qw": 127,
                "sigma_x": 10,
                "sigma_y": 10,
                "sigma_z": 10,
                "sigma_roll": 30,
                "sigma_pitch": 30,
                "sigma_yaw": 30,
                "range": 10,
            }
        }
        # Variables for external sensors
        self.imu_relative_poses = {}
        self.gps_fix = [[],[],[]] # [position, orientation, covariance]
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
        rospy.wait_for_service("modem/ping_modem")
        self.ping_client = rospy.ServiceProxy("modem/ping_modem", PingModem)
        self.tdma_from_modem = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_status)
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        self.nmea_to_modem = rospy.Publisher("modem/nmea_to_modem", String, queue_size=10) # Not actually used
        # Establish the message subs and pubs
        self.init_prior_sub = rospy.Subscriber("from_acomms/init_prior", InitPrior, self.on_init_prior)
        self.partial_graph_sub = rospy.Subscriber("from_acomms/partial_graph", PartialGraph, self.on_partial_graph)
        self.init_prior_pub = rospy.Publisher("to_acomms/init_prior", InitPrior, queue_size=1)
        self.partial_graph_pub = rospy.Publisher("to_acomms/partial_graph", PartialGraph, queue_size=1)
        # Initialize Subscribers for handling external sensors
        self.depth = rospy.Subscriber("depth", Float32, self.on_depth)
        self.gps = rospy.Subscriber("gps", PoseWithCovarianceStamped, self.on_gps)
        self.preintegrated_imu = rospy.Subscriber("preintegrated_imu", PoseWithCovarianceStamped, self.on_preintegrated_imu)
        # Initialize the pubs/subs for create relative pose measurements from the estimator
        self.acomms_event = rospy.Publisher("acomms_event", Time, queue_size=1)
        self.cycle_graph_pub = rospy.Publisher("cycle_graph", CycleGraph, queue_size=1)
        # Initialize the modem addresses and cycle targets
        self.setup_addresses()
        self.setup_cycle_targets()
        # Start the cycle
        rospy.loginfo("[%s] Starting Comms Cycle" % rospy.Time.now())

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
        
        # Vify that the number of agents and landmarks are within valid ranges
        if not (1 <= self.num_agents <= 3):
            rospy.logerr("[%s] Invalid number of agents: %d" % (rospy.Time.now(), self.num_agents))
            return
        if not (0 <= self.num_landmarks <= 2):
            rospy.logerr("[%s] Invalid number of landmarks: %d" % (rospy.Time.now(), self.num_landmarks))
            return

        # Determine active TDMA slots based on number of agents
        expected_slots = 6
        active_slots = rospy.get_param("~active_slots", "0,3")
        # Actice slots is a string of comma-separated integers, so we need to convert it to a list of integers
        active_slot_list = active_slots.split(",")
        active_slots = [int(slot) for slot in active_slot_list if slot.isdigit() and int(slot) < expected_slots]
        rospy.loginfo("[%s] Active Slots: %s" % (rospy.Time.now(), active_slots))
        
        # Build a list of the target agents addresses from modem_addresses
        tgt_agents = [self.modem_addresses[chr(ord('A') + i)][0] for i in range(self.num_agents) if i != self.local_address]
        # Build a list of target landmark addresses
        tgt_ldmks = [self.modem_addresses["L%d" % i][0] for i in range(self.num_landmarks)]
        # Generate cycle target mapping
        for i in range(expected_slots):
            # active slots is list of integers, so we can check if i is in the list
            if i in active_slots:
                # Ensure we don't attempt modulo with empty lists
                agent_target = tgt_agents[i % len(tgt_agents)] if tgt_agents else None
                ldmk_target = tgt_ldmks[i % len(tgt_ldmks)] if tgt_ldmks else None
                self.cycle_target_mapping[i] = [agent_target, ldmk_target]
            else:
                self.cycle_target_mapping[i] = [[],[]]

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
        # Check if the TDMA cycle is restarting:
        if current_slot == 0:
            self.tdma_cycle_sequence += 1
            self.on_tdma_cycle_reset()

        # Load the cycle message into the queue so its ready to go when we are active
        if time_to_next_active < self.msg_preload and self.num_agents > 1:
            # If we are in the preload window, load one of two possible messages to the queue so its ready to go
            if self.tdma_cycle_sequence == 0:
                self.init_prior_pub.publish(self.staged_init_prior)
            else:
                self.partial_graph_pub.publish(self.staged_partial_graph)
        elif we_are_active == True:
            if self.tdma_status.we_are_active == False:
                # The TDMA cycle has just started, clear the ping safety
                self.ping_slot_open = True
                rospy.loginfo("[%s] TDMA Active Slot Started, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
            # If we are active and we've cleared the preloaded message
            elif elapsed_time_in_slot > 2 and self.ping_slot_open == True: # ping safety used to ensure this happens only once
                # Execute the ping cycle for the current TDMA slot
                self.execute_ping_cycle(current_slot, remaining_active_sec)
            elif elapsed_time_in_slot > 2 + self.ping_timeout and self.ping_slot_open == True:
                # If we have a ping timeout, reset the ping safety
                self.execute_ping_cycle(current_slot, remaining_active_sec)
            elif elapsed_time_in_slot % 1 == 0 and self.ping_slot_open == False:
                # If we are active but not in the ping window, do nothing
                rospy.loginfo("[%s] In Active TDMA Slot, waiting for ping window" % rospy.Time.now())
        else:
            # If we are not active, do nothing
            pass

        # Regardless, update TDMA
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
        if len(self.inbound_partial_graphs.keys()) > self.num_agents:
            # We probably synched this and sent it to the estimator already, clear the cycle data
            self.cycle_graph_data.clear()
        else:
            rospy.logwarn("[%s] Not enough partial graphs received to trigger synchronization" % rospy.Time.now())
            # TODO: Add a function to handle the case that only one or two partial graphs were recieved, but not enough to trigger synchronization
            self.synchronize_partial_graphs()
            self.build_cycle_graph()
            # Try synchronizing what we have as the cycle expires, then clear
            self.cycle_graph_data.clear()

        # Check that the partial graph data we've generated is valid, then build the partial graph message
        num_btwn, num_rng_from_us, num_rng_to_us = self.check_partial_graph_data()
        self.staged_partial_graph = self.build_partial_graph_from_local_data()
        # Clear the partial graph data to prepare for the next cycle
        self.partial_graph_data.clear()

        # If the init prior dict is not full, reset the tdma_cycle sequence so we keep trying to initialize
        if len(self.inbound_init_priors.keys()) < (self.num_agents + self.num_landmarks):
            self.tdma_cycle_sequence = 0
            # Get the first and last pose keys in the self.staged_partial_graph
            first_key = chr(ord("A") + self.local_address) + "0"
            last_key = chr(ord("A") + self.local_address) + str(self.staged_partial_graph.full_index + self.staged_partial_graph.num_poses)
            # Integrate across the poses to get the relative pose from first_key to last_key
            position, orientation, sigmas = self.integrate_across_poses(first_key, last_key)
            self.failed_cycle_relative_pose["key1"] = first_key
            self.failed_cycle_relative_pose["key2"] = last_key
            self.failed_cycle_relative_pose["position"] = position
            self.failed_cycle_relative_pose["orientation"] = orientation
            self.failed_cycle_relative_pose["sigmas"] = sigmas
            # Reset the local pose index to 0 for the next cycle (preserves the partial graph format)
            self.modem_addresses[chr(ord("A") + self.local_address)][1] = 0
            rospy.logwarn("[%s] TDMA Cycle Reset, waiting for init prior data" % rospy.Time.now())
            return
        else:
            self.tdma_cycle_sequence += 1
        return
    
    def execute_ping_cycle(self, current_slot, remaining_active_sec):
        """This function executes the cycle for the current TDMA slot
        - Assumes that there are 6 slots in the TDMA cycle and we only ping one agent and one landmark in each slot
        """
        # Get the targets from the cycle target mapping
        tgt_agent, tgt_ldmk = self.cycle_target_mapping[current_slot]
        #rospy.loginfo("[%s] Current TDMA Slot: %d, TGT Agent: %d, TGT Landmark: %s" % (rospy.Time.now(), current_slot, tgt_agent, tgt_ldmk))
        elapsed_time_in_slot = self.tdma_status.slot_duration_seconds - remaining_active_sec
        preload_buffer = 1 # seconds to allow for clearing the preloading message (to prevent both the ping and msg from being sent at the same time)
        self.ping_slot_open = False # Reset the ping safety
        # Ping the agent
        if tgt_agent != None and elapsed_time_in_slot > preload_buffer:
            self.send_ping(tgt_agent)
            rospy.loginfo("[%s] Sent Ping to Agent %s" % (rospy.Time.now(), tgt_agent))
        # Ping the landmark (if we pinged an agent in the first slot)
        elif (tgt_agent != None and tgt_ldmk != None) and elapsed_time_in_slot > preload_buffer + self.ping_timeout:
            self.send_ping(tgt_ldmk)
            rospy.loginfo("[%s] Sent Ping to Landmark %s" % (rospy.Time.now(), tgt_ldmk))
        # Ping the landmark (if we don't have an agent to ping)
        elif (tgt_agent == None and tgt_ldmk != None) and elapsed_time_in_slot > preload_buffer:
            self.send_ping(tgt_ldmk)
            rospy.loginfo("[%s] Sent Ping to Landmark %s" % (rospy.Time.now(), tgt_ldmk))
        elif (tgt_agent == None and tgt_ldmk != None) and elapsed_time_in_slot > preload_buffer + self.ping_timeout:
            # Shift the tgt_ldmk to the other ldmk
            new_tgt_ldmk = tgt_ldmk + 1 if tgt_ldmk < (self.num_agents + self.num_landmarks) else tgt_ldmk -1
            self.send_ping(new_tgt_ldmk)
            rospy.loginfo("[%s] Sent Ping to Landmark %s" % (rospy.Time.now(), new_tgt_ldmk))
        else:
            rospy.loginfo("[%s] No ping to send in this slot" % rospy.Time.now())
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
                self.acomms_event.publish(acomms_event_time)
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
                self.acomms_event.publish(timestamp)
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
    def on_preintegrated_imu(self, msg: PoseWithCovarianceStamped):
        """This function receives the preintegrated IMU data from the estimator
        Args:
            msg (PoseWithCovarianceStamped): The preintegrated IMU data
        """
        # Get the current time
        pose_time = msg.header.stamp
        # Get the pose index for the local agent by calulating the letter from the index
        local_chr = chr(ord("A") + self.local_address)
        key1_index = self.modem_addresses[local_chr][1]
        key1 = local_chr + str(key1_index)
        key2 = local_chr + str(key1_index+1)
        # Update index (only place where this is done)
        self.modem_addresses[local_chr][1] = key1_index + 1
        self.pose_time_lookup[key2] = pose_time
        # Get the relative pose data
        pose = msg.pose.pose
        position = pose.position
        orientation = pose.orientation
        covariance = msg.pose.covariance
        sigmas = np.sqrt(np.diag(covariance))
        # Store the relative pose data in the imu_relative_poses dict
        self.imu_relative_poses[pose_time] = {
            "key1": key1,
            "key2": key2,
            "position": np.array([position.x, position.y, position.z]),
            "orientation": np.array([orientation.x, orientation.y, orientation.z, orientation.w]),
            "sigmas": sigmas
        }
        if self.failed_cycle_relative_pose["key1"] is not None and self.failed_cycle_relative_pose["key2"] is not None:
            # If we have a failed cycle relative pose compose the existing pose with the new relative pose
            existing_orientation = spt.Rotation.from_quat(self.failed_cycle_relative_pose["orientation"])
            existing_covariance = np.diag(self.failed_cycle_relative_pose["sigmas"]**2)
            new_position = self.failed_cycle_relative_pose["position"] + existing_orientation.apply(np.array([position.x, position.y, position.z]))
            new_orientation = existing_orientation * spt.Rotation.from_quat(np.array([orientation.x, orientation.y, orientation.z, orientation.w]))
            new_covariance = existing_covariance + (covariance @ covariance.T)  # Propagate covariance
            new_sigmas = np.sqrt(np.diag(new_covariance))
            # add to the cycle graph data
            self.partial_graph_data["BTWN_%s_%s" % (self.failed_cycle_relative_pose["key1"], key2)] = {
                "key1": self.failed_cycle_relative_pose["key1"],
                "key2": key2,
                "position": new_position,
                "orientation": new_orientation.as_quat(),
                "sigmas": new_sigmas
            }
        else:
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
        self.imu_relative_poses = {time: data for time, data in self.imu_relative_poses.items() if time >= (pose_time - rospy.Duration(180))}
        return

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
    
    def on_depth(self, msg: Float32):
        """This function receives the depth data from the estimator
        Args:
            msg (Float32): The depth data
        """
        # Get the current time
        pose_time = rospy.Time.now()
        # Replace the initial_depth with the depth data
        # Log the reciept
        #rospy.loginfo("[%s] Received Depth data: %f, Modem is on %s" % (pose_time, msg.data, "safe" if self.modem_on_safe else "unsafe"))
        return
    
    # Pre-encoding and decoding functions:
    def encode_init_prior_data_as_int(self, initial_position, initial_orientation, initial_sigmas):
        """This function encodes the initial prior factor data into a message
        Args:
            initial_position (np.array): The initial position
            initial_orientation (np.array): The initial orientation
            initial_sigmas (np.array): The initial sigmas
        Returns:
            PosePriorFactor: The encoded initial prior factor message
        """
        # Position is a int16[3], multiply by scale, then convert to int16
        x = int(initial_position[0] * self.codec_scale_factors["init_prior"]["x"])
        y = int(initial_position[1] * self.codec_scale_factors["init_prior"]["y"])
        z = int(initial_position[2] * self.codec_scale_factors["init_prior"]["z"])
        # verify all fields are within int16 range
        if not (np.all(np.abs([x, y, z]) <= 32767)):
            rospy.logerr("[%s] Initial position values out of range!" % rospy.Time.now())
            return None
        else:
            initial_position = np.array([x, y, z], dtype=np.int16)

        # Orientation is a int16[4] (quaternion)
        qx = int(initial_orientation[0] * self.codec_scale_factors["init_prior"]["qx"])
        qy = int(initial_orientation[1] * self.codec_scale_factors["init_prior"]["qy"])
        qz = int(initial_orientation[2] * self.codec_scale_factors["init_prior"]["qz"])
        qw = int(initial_orientation[3] * self.codec_scale_factors["init_prior"]["qw"])
        # verify all fields are within int16 range
        if not (np.all(np.abs([qx, qy, qz, qw]) <= 32767)):
            rospy.logerr("[%s] Initial orientation values out of range!" % rospy.Time.now())
            return None
        else:
            initial_orientation = np.array([qx, qy, qz, qw], dtype=np.int16)

        # Sigmas is a int16[6], multiply by scale, then convert to int16
        sx = int(initial_sigmas[0] * self.codec_scale_factors["init_prior"]["sigma_x"])
        sy = int(initial_sigmas[1] * self.codec_scale_factors["init_prior"]["sigma_y"])
        sz = int(initial_sigmas[2] * self.codec_scale_factors["init_prior"]["sigma_z"])
        sroll = int(initial_sigmas[3] * self.codec_scale_factors["init_prior"]["sigma_roll"])
        spitch = int(initial_sigmas[4] * self.codec_scale_factors["init_prior"]["sigma_pitch"])
        syaw = int(initial_sigmas[5] * self.codec_scale_factors["init_prior"]["sigma_yaw"])
        # verify all fields are within int16 range
        if not (np.all(np.abs([sx, sy, sz, sroll, spitch, syaw]) <= 32767)):
            rospy.logerr("[%s] Initial sigmas values out of range!" % rospy.Time.now())
            return None
        else:
            initial_sigmas = np.array([sx, sy, sz, sroll, spitch, syaw], dtype=np.int16)
        return initial_position, initial_orientation, initial_sigmas
    
    def decode_init_prior_data_from_int(self, initial_position, initial_orientation, initial_sigmas):
        # Apply the reverse of the encoding process to decode the data
        # Position is a int16[3], divide by scale to get original value
        initial_position = np.array([
            initial_position[0] / self.codec_scale_factors["init_prior"]["x"],
            initial_position[1] / self.codec_scale_factors["init_prior"]["y"],
            initial_position[2] / self.codec_scale_factors["init_prior"]["z"]
        ])
        # Orientation is a int16[4] (quaternion), divide by scale to get original value
        initial_orientation = np.array([
            initial_orientation[0] / self.codec_scale_factors["init_prior"]["qx"],
            initial_orientation[1] / self.codec_scale_factors["init_prior"]["qy"],
            initial_orientation[2] / self.codec_scale_factors["init_prior"]["qz"],
            initial_orientation[3] / self.codec_scale_factors["init_prior"]["qw"]
        ])
        # Sigmas is a int16[6], divide by scale to get original value
        initial_sigmas = np.array([
            initial_sigmas[0] / self.codec_scale_factors["init_prior"]["sigma_x"],
            initial_sigmas[1] / self.codec_scale_factors["init_prior"]["sigma_y"],
            initial_sigmas[2] / self.codec_scale_factors["init_prior"]["sigma_z"],
            initial_sigmas[3] / self.codec_scale_factors["init_prior"]["sigma_roll"],
            initial_sigmas[4] / self.codec_scale_factors["init_prior"]["sigma_pitch"],
            initial_sigmas[5] / self.codec_scale_factors["init_prior"]["sigma_yaw"]
        ])
        return initial_position, initial_orientation, initial_sigmas
    
    def encode_partial_graph_data_as_int(self, id, position, orientation, sigmas):
        """This function encodes the partial graph data into a message
        Args:
            position (np.array): The relative position
            orientation (np.array): The relative orientation
            sigmas (np.array): The relative sigmas
        Returns:
            PartialGraph: The encoded partial graph message
        """
        if id == 0:
            # Position is a int16[3], multiply by scale, then convert to int16
            x = int(position[0] * self.codec_scale_factors["partial_graph"]["x_0"])
            y = int(position[1] * self.codec_scale_factors["partial_graph"]["y_0"])
            z = int(position[2] * self.codec_scale_factors["partial_graph"]["z_0"])
            # verify all fields are within int16 range
            if not (np.all(np.abs([x, y, z]) <= 32767)):
                rospy.logerr("[%s] Initial position values out of range!" % rospy.Time.now())
                return None
            else:
                position = list([x, y, z])
        else:
            # Position is a int8[3], multiply by scale, then convert to int8
            x = int(position[0] * self.codec_scale_factors["partial_graph"]["x"])
            y = int(position[1] * self.codec_scale_factors["partial_graph"]["y"])
            z = int(position[2] * self.codec_scale_factors["partial_graph"]["z"])
            # verify all fields are within int8 range
            if not (np.all(np.abs([x, y, z]) <= 127)):
                rospy.logerr("[%s] Relative position values out of range!" % rospy.Time.now())
                return None
            else:
                position = list([x, y, z])

        # Orientation is a int8[4] (quaternion), multiply by scale, then convert to int8
        qx = int(orientation[0] * self.codec_scale_factors["partial_graph"]["qx"])
        qy = int(orientation[1] * self.codec_scale_factors["partial_graph"]["qy"])
        qz = int(orientation[2] * self.codec_scale_factors["partial_graph"]["qz"])
        qw = int(orientation[3] * self.codec_scale_factors["partial_graph"]["qw"])
        # verify all fields are within int8 range
        if not (np.all(np.abs([qx, qy, qz, qw]) <= 127)):
            rospy.logerr("[%s] Relative orientation values out of range!" % rospy.Time.now())
            return None
        else:
            orientation = list([qx, qy, qz, qw])

        # Sigmas is a int8[6], multiply by scale, then convert to int8
        sx = int(sigmas[0] * self.codec_scale_factors["partial_graph"]["sigma_x"])
        sy = int(sigmas[1] * self.codec_scale_factors["partial_graph"]["sigma_y"])
        sz = int(sigmas[2] * self.codec_scale_factors["partial_graph"]["sigma_z"])
        sroll = int(sigmas[3] * self.codec_scale_factors["partial_graph"]["sigma_roll"])
        spitch = int(sigmas[4] * self.codec_scale_factors["partial_graph"]["sigma_pitch"])
        syaw = int(sigmas[5] * self.codec_scale_factors["partial_graph"]["sigma_yaw"])
        # verify all fields are within int8 range
        if not (np.all(np.abs([sx, sy, sz, sroll, spitch, syaw]) <= 255)):
            rospy.logerr("[%s] Relative sigmas values out of range!" % rospy.Time.now())
            return None
        else:
            sigmas = list([sx, sy, syaw])
        return position, orientation, sigmas
    
    def decode_partial_graph_data_from_int(self, position, orientation, sigmas):
        """This function decodes the partial graph data from a message
        Args:
            position (list): The relative position
            orientation (list): The relative orientation
            sigmas (list): The relative sigmas
        Returns:
            tuple: (position, orientation, sigmas)
        """
        # Decode the data by applying the reverse of the encoding process
        # Position is a int16[3], divide by scale to get original value
        position = np.array([
            position[0] / self.codec_scale_factors["partial_graph"]["x"],
            position[1] / self.codec_scale_factors["partial_graph"]["y"],
            position[2] / self.codec_scale_factors["partial_graph"]["z"]
        ])
        
        # Orientation is a int8[4], divide by scale to get original value
        orientation = np.array([
            orientation[0] / self.codec_scale_factors["partial_graph"]["qx"],
            orientation[1] / self.codec_scale_factors["partial_graph"]["qy"],
            orientation[2] / self.codec_scale_factors["partial_graph"]["qz"],
            orientation[3] / self.codec_scale_factors["partial_graph"]["qw"]
        ])
        
        # Sigmas is a int8[6], divide by scale to get original value
        sigmas = np.array([
            sigmas[0] / self.codec_scale_factors["partial_graph"]["sigma_x"],
            sigmas[1] / self.codec_scale_factors["partial_graph"]["sigma_y"],
            self.sigma_depth,
            self.sigma_roll,
            self.sigma_pitch,
            sigmas[5] / self.codec_scale_factors["partial_graph"]["sigma_yaw"]
        ])
        
        return position, orientation, sigmas
    
    # Message Processing Functions
    def check_partial_graph_data(self):
        """This function checks the cycle graph data for completeness and validity
        Returns:
            bool: True if the cycle graph data is valid, False otherwise
        """
        # Check that the cycle data is not empty
        if not self.partial_graph_data:
            rospy.logerr("[%s] Cycle graph data is empty" % rospy.Time.now())
            return 0,0,0
        # Check the number of BTWN entries (should be between 0 and 6)
        num_btwn = len([key for key in self.partial_graph_data.keys() if key.startswith("BTWN")])
        if num_btwn < 0 or num_btwn > 6:
            rospy.logerr("[%s] Invalid number of BTWN entries: %d" % (rospy.Time.now(), num_btwn))
            return False  
        # Check the number of RNG entries with range data
        num_rng_from_us = len([key for key in self.partial_graph_data.keys() if key.startswith("RNG") and self.partial_graph_data[key].get("range") is not None])
        if num_rng_from_us < 0 or num_rng_from_us > 4:
            rospy.logerr("[%s] Invalid number of RNG entries with range data: %d" % (rospy.Time.now(), num_rng_from_us))
            return False
        # Check the number of RNG entries without range data (should be between 0 and 2)
        num_rng_to_us = len([key for key in self.partial_graph_data.keys() if key.startswith("RNG") and self.partial_graph_data[key].get("range") is None])
        if num_rng_to_us < 0 or num_rng_to_us > 2:
            rospy.logerr("[%s] Invalid number of RNG entries without range data: %d" % (rospy.Time.now(), num_rng_to_us))
            return False
        # Check that for the key "BTWN_key1_key2" the key1 and key2 are sequential and that the key2 of one looks like the key1 of the next
        local_pose_keys = []
        for i in range(num_btwn - 1):
            key1 = "BTWN_%s_%s" % (chr(ord("A") + self.local_address), i)
            key2 = "BTWN_%s_%s" % (chr(ord("A") + self.local_address), i + 1)
            if key2 not in self.partial_graph_data:
                rospy.logerr("[%s] Missing key2: %s" % (rospy.Time.now(), key2))
                return False
            if self.partial_graph_data[key1]["key2"] != self.partial_graph_data[key2]["key1"]:
                rospy.logerr("[%s] Key mismatch between %s and %s" % (rospy.Time.now(), key1, key2))
                return False
            local_pose_keys.append(key1)
        # Check that the RNG entries each include a key1 that is in the BTWN entries
        for key in self.partial_graph_data.keys():
            if key.startswith("RNG"):
                if self.partial_graph_data[key]["key1"] not in local_pose_keys:
                    rospy.logerr("[%s] RNG entry %s does not match any BTWN entry" % (rospy.Time.now(), key))
                    return False
        return num_btwn, num_rng_from_us, num_rng_to_us

    def process_partial_graph(self, msg:PartialGraph):
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
        """This function synchronizes the partial graphs once all three agents have reported
        - It assumes that process_partial_graph has been called for each agent's partial graph
        - It associates the ranges between graphs, checks connectivity and adds range sigmas
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
        poses = []
        for key in self.cycle_graph_data.keys():
            if key.startswith("BTWN"):
                key1 = self.cycle_graph_data[key]["key1"]
                key2 = self.cycle_graph_data[key]["key2"]
                if key1 not in poses:
                    poses.append(key1)
                if key2 not in poses:
                    poses.append(key2)

        # Check that the ranges are associated with a pose and contain a measurement
        for key in self.cycle_graph_data.keys():
            if key.startswith("RNG"):
                key1_check = True
                key2_check = True
                range_check = True
                # Check that the key1 is in the poses list
                if self.cycle_graph_data[key]["key1"] not in poses:
                    key1_check = False
                    continue
                    # Check that the key2 is in the poses list
                if self.cycle_graph_data[key]["key2"] not in poses:
                    key2_check = False
                    continue
                # Check that the range is not None
                if self.cycle_graph_data[key]["range"] is None:
                    range_check = False
                    continue
                # If all checks pass, add the range to the cycle graph data
                if key1_check and key2_check and range_check:
                    self.cycle_graph_data[key]["range_sigma"] = self.sigma_range
                    continue
                elif (not key1_check or not key2_check) and range_check:
                    rospy.logerr("[%s] Recieved Range Failed to Associate" % (rospy.Time.now(), key))
                    continue
                elif (key1_check and key2_check) and not range_check:
                    rospy.logerr("[%s] Transmitted Range Failed to Associate" % (rospy.Time.now(), key))
                    continue
                elif not key1_check and not key2_check :
                    rospy.loginfo("[%s] Range entry %s has no associated in poses" % (rospy.Time.now(), key))
                    del self.cycle_graph_data[key]
                    continue
                else:
                    rospy.logerr("[%s] Range entry %s is incomplete!" % (rospy.Time.now(), key))
                    continue
    
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
        initial_position, initial_orientation, initial_sigmas = self.encode_init_prior_data_as_int([0,0,0], [0,0,0,1], [1.7, 1.7, 0.1, 0.5, 0.5, 0.5])
        #initial_position, initial_orientation, initial_sigmas = self.encode_init_prior_data_as_int(self.gps_fix[0], self.gps_fix[1], self.gps_fix[2])
        if initial_position is None or initial_orientation is None or initial_sigmas is None:
            rospy.logerr("[%s] Failed to encode initial prior factor data!" % rospy.Time.now())
            return
        # Set the initial prior factor message
        init_prior_msg = InitPrior()
        init_prior_msg.local_addr = int(self.local_address)
        init_prior_msg.full_index = int(self.modem_addresses[local_symbol][1])
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
        Returns:
            PartialGraph: The built partial graph message
        """
        # Initialize the partial graph message
        partial_graph = PartialGraph()
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
                # Extract the relative position and orientation data
                position, orientation, sigmas = self.encode_partial_graph_data_as_int(num_btwn, data["position"], data["orientation"], data["sigmas"])
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
            # Pass the data to the cycle graph data for encoding
            self.cycle_graph_data = data
            self.inbound_partial_graphs[chr(self.local_address + ord("A"))] = data
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
                position, orientation, sigmas = self.encode_partial_graph_data_as_int(0, data["position"], data["orientation"], data["sigmas"])
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
            # TODO: Cue the comms cycle to start for real by incrementing the tdma_cycle sequence the next time the cycle rolls
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
            self.inbound_partial_graphs[local_addr] = msg
            self.process_partial_graph(msg)
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
            rospy.logerr("[%s] Received duplicate partial graph from %s" % (rospy.Time.now(), local_addr))
        return 
    
    # TODO: Revise the partial/cycle graph handling so that the data structures support 1:1 - 3:2 agent/landmark scenarios
    # TODO: Write a function to synchronize/clean the graph if only one or two are present (triggered by tdma roll)
    # - I think this works for one graph (but I should check), but probably not if you were expecteding 3 and got one.
    
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