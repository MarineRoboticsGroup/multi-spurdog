#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_acomms_msgs.msg import(
    TdmaStatus, TdmaAdvancedStatus
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest,
)
from spurdog_acomms.msg import(
    PosePriorFactor, PartialGraph, RelativePoseMin, InitiatedRange, RecievedRange
)
from std_msgs.msg import String, Time

class CycleManager:
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.local_address = rospy.get_param("modem_address", 0)
        self.num_agents = rospy.get_param("num_agents", 1)
        self.num_landmarks = rospy.get_param("num_landmarks", 0)
        self.landmarks = rospy.get_param("landmarks", {}) # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = rospy.get_param("sound_speed", 1500)
        self.sigma_range = rospy.get_param("sigma_range", 1)
        self.sigma_depth = rospy.get_param("sigma_depth", 1)
        self.sigma_roll = rospy.get_param("sigma_roll", 0.1)
        self.sigma_pitch = rospy.get_param("sigma_pitch", 0.1)
        self.initial_gps_fix = None
        # Cycle Tracking Flags
        self.run_cycle = False
        self.tdma_status = TdmaStatus()
        self.tdma_cycle_sequence = 0
        self.cycle_status = 0
        self.cycle_tgt_agent = None
        self.cycle_tgt_ldmk = None
        self.priors_recieved = 0
        # Ping Tracking
        self.ping_sent_time = 0
        self.acomms_event_time = 0
        # TODO: Pose Indexing
        self.modem_addresses = {} # [address, pose_index]
        # Maybe restructure this to be a dictionary of dictionaries
        self.pose_time_lookup = {} # {key: time}, supports factor-to-time mapping
        self.partial_graph_data = {}
        self.comms_cycle_data = {}
        # Message compression
        self.prior_scale_factors = { # The factors we multiply by to scale the data to int16 or int8
            "x_scale": 10,          #int16
            "y_scale": 10,          #int16
            "z_scale": 1000,        #int16
            "qx_scale": 32000,      #int16
            "qy_scale": 32000,      #int16
            "qz_scale": 32000,      #int16
            "qw_scale": 32000,      #int16
            "sigma_x_scale": 10,    #uint8
            "sigma_y_scale": 10,    #uint8
            "sigma_z_scale": 10,    #uint8
            "sigma_roll_scale": 10, #uint8
            "sigma_pitch_scale": 10,#uint8
            "sigma_yaw_scale": 10,  #uint8
        }
        self.partial_graph_scale_factors = { # The factors we multiply by to scale the data to int16 or int8
            "dx_scale": 10,         #int8
            "dy_scale": 10,         #int8
            "dz_scale": 10,         #int8
            "dqx_scale": 100,       #int8
            "dqy_scale": 100,       #int8
            "dqz_scale": 100,       #int8
            "dqw_scale": 100,       #int8
            "sigma_x_scale": 10,    #uint8
            "sigma_y_scale": 10,    #uint8
            "sigma_z_scale": 10,    #uint8
            "sigma_roll_scale": 10, #uint8
            "sigma_pitch_scale": 10,#uint8
            "sigma_yaw_scale": 10,  #uint8
            "sigma_range_scale": 10,#uint8
            "range_scale": 10,      #int16
        }

        # Check services, then initialize them
        rospy.wait_for_service("modem/PingModem")
        self.ping_client = rospy.ServiceProxy("modem/PingModem", PingModem)
        # Monitor modem TDMA status to control the comms cycle
        self.tdma_status = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_status)
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        
        # Initialize the pubs/subs for create relative pose measurements from the estimator
        self.acomms_event = rospy.Publisher("acomms_event", Time, queue_size=1)
        self.preintegrated_imu = rospy.Subscriber("preintegrated_imu", PoseWithCovarianceStamped, self.on_preintegrated_imu)

        # TODO: Add a way to pass the graph data to the estimator
        
        # Setup the publishers for the message queues
        # For each agent (not including us), creata unique publisher for PosePriorFactor and PartialGraph
        self.init_prior_pubs, self.init_prior_subs = {}
        self.partial_graph_pubs, self.partial_graph_subs = {}
        for key in self.modem_addresses:
            if key[0] != "L" or key[1] != self.local_address:
                self.init_prior_pubs[key] = rospy.Publisher("modem/init_prior_%s" % key, PosePriorFactor, queue_size=10)
                self.partial_graph_pubs[key] = rospy.Publisher("modem/partial_graph_%s" % key, PartialGraph, queue_size=10)
        for key in self.modem_addresses:
            if key[0] != "L" and key[1] != self.local_address:
                self.init_prior_subs[key] = rospy.Subscriber("modem/init_prior_%s" % key, PosePriorFactor, self.on_init_prior)
                self.partial_graph_subs[key] = rospy.Subscriber("modem/partial_graph_%s" % key, PartialGraph, self.on_partial_graph)
    
    # Setup Functions:
    def setup_addresses(self):
        """This function sets up the number of agents and landmarks
        """
        # Build the modem addresses
        if self.num_agents == 1 or self.num_landmarks == 0:
            rospy.logerr("[%s] No one to ping!" % rospy.Time.now())
        else:
            # Build the key tracker, assigning a unique integer to each agent and landmark
            for i in range(self.num_agents):
                letter = chr(ord('A') + i)
                if address == self.local_address:
                    # Assigns an integer for indexing our own poses
                    self.modem_addresses[letter] = [i, 0]
                else:
                    self.modem_addresses[letter] = [i]
            for i in range(self.num_landmarks):
                address = i + self.num_landmarks
                self.modem_addresses["L%d" % i] = [address,0]
        return
    
    def initialize_graph_data_structure(self, num_btwn=0, num_rng=0, num_pri=0):
        """
        Initializes a dictionary where each entry has a unique key and follows 
        one of three templates: BTWN, RNG, or PRI.
        - This is valuable, because it can be initialized differently for different cycles
        and be used to store both the partial graph and the full graph data.
        Args:
            num_btwn (int): Number of BTWN entries to create.
            num_rng (int): Number of RNG entries to create.
            num_pri (int): Number of PRI entries to create.

        Returns:
            dict: A structured dictionary with unique keys for each entry.
        """
        graph_dict = {}

        for i in range(num_btwn):
            graph_dict[f"BTWN_{i+1}"] = {
                "time": None, "key1": None, "key2": None, 
                "position": None, "orientation": None, "noise_sigmas": None
            }
        
        for i in range(num_rng):
            graph_dict[f"RNG_{i+1}"] = {
                "time": None, "key1": None, "key2": None, 
                "measured_range": None, "sigma_range": None,
            }

        for i in range(num_pri):
            graph_dict[f"PRI_{i+1}"] = {
                "time": None, "key1": None, 
                "position": None, "orientation": None, "noise_sigmas": None
            }
        #self.partial_graph_data = partial_graph
        return graph_dict
    
    def generate_ldmk_priors(self):
        # for each landmark, generate a prior and add it to the partial graph
        for key, value in self.landmarks.items():
            # Get the current time
            time = rospy.Time.now()
            # Get the position and orientation
            position = value
            orientation = [0, 0, 0, 1]
            noise = [1.7, 1.7, 4.3, 0.1, 0.1, 0.1]
            # Get the next available PRI key
            pri_key = self.get_next_graph_key(self.partial_graph_data, "PRI")
            # Add this to the partial graph data
            self.partial_graph_data[pri_key] = {
                "time": time,
                "key1": key,
                "position": position,
                "orientation": orientation,
                "noise_sigmas": noise
            }
        return
    
    def get_next_graph_key(self, graph_dict, type):
        """This function returns the next available key for the partial graph data
        Args:
            graph_dict (dict): The partial graph data structure or comms cycle data structure
            type (str): The type of key to return (BTWN, RNG, PRI)
        """
        # Find the next empty RNG key in partial graph data
        next_key = None
        for key in graph_dict.keys():
            if key.startswith(type) and graph_dict[key]["time"] is None:
                next_key = key
                break

        if next_key is None:
            rospy.logwarn("No available slots in graph data structure!")
        return next_key
    
    def on_startup(self):
        """This function is called on startup to initialize the cycle manager
        - Assumes (somehow) that the node has the GPS position
        - Assumes that the sending of the initial GPS prior is done by the cycle manager
        """
        # Setup the modem addresses
        self.setup_addresses()
        # Initialize the graph data structure to take the cycle init packets
        self.partial_graph_data = self.initialize_graph_data_structure(
                        num_btwn=(self.num_agents-1)+self.num_landmarks,
                        num_rng=(self.num_agents-1)+self.num_landmarks,
                        num_pri=self.num_agents+self.num_landmarks
                    )
        if self.num_landmarks > 0:
            self.generate_ldmk_priors()
        return
    
    # TDMA Cycle Tracking:
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
        
        # Check if the TDMA slot has changed:
        if current_slot != self.tdma_status.current_slot:
            # If we are at the start of a new cycle, save the partial graph data and reset it
            if current_slot == 0:
                # Check if the partial graph from the last cycle is complete
                if self.check_partial_graph(): # This also adds it to the comms_cycle_data
                   rospy.loginfo("[%s] Partial Graph Complete" % rospy.Time.now()) 
                else:
                    rospy.logwarn("[%s] Partial Graph Incomplete" % rospy.Time.now())
                
                #NOTE: This ensures that the tdma_cycle_sequence is reset after we recieve the priors from the other agents
                # s.t. the following 2 cycles are logged to the partial graph, then closed at the start of the 3rd cycle
                # Trigggering it here, after the slot has changed, ensures it happens at the same time on all 3 agents
                # TODO: This is still vulnerable to the robots entering the water asynchronously: it assumes that they all transmit their priors in the same TDMA cycle
                # This would be a really inconvenient failure mode, so I should work out whether they are required at all (given CORA). I might be able
                # to use some sort of arbitrary first pose.
                # TODO: But functionally, you could probably just put some sort of depth safety, and as long as they all make it on depth within 45sec, they should be fine.
                # The problem is that the current between factor scaling doesn't really allow for that (it's capped at 12.7m), so I would need to change that.
                # A scale factor of 3 (vs. 10) would allow for 42m, which is probably enough, but results in very low accuracy.
                # A better alternative would be to include a between factor in that first message, that can support the longer pose delta
                # - You would have to extract a relative pose from GPS fix to the start of the relevant comms cycle.
                # - There a question of how you would time it:
                # -- Should look for a software mute light, because I think you could use the light on one of them to time it.
                # -- Otherwise, I think you'd need to have one agent (src=0) ping periodically on the begining of the cycle so you can time the launch.

                # If the tdma_cycle sequence is even, we are entering the next cycle and need to batch the graph
                if self.tdma_cycle_sequence % 2 == 0 and self.tdma_cycle_sequence > 0 and self.priors_recieved >= self.num_agents-1:
                    # Reset the partial graph data
                    self.partial_graph_data = self.initialize_graph_data_structure(
                        num_btwn=(self.num_agents-1)+self.num_landmarks,
                        num_rng=(self.num_agents-1)+self.num_landmarks,
                        num_pri=0
                    )
                # If we aren't on an even cycle, but we have recieved all the priors, we just need to increment the cycle
                elif self.priors_recieved >= self.num_agents-1:
                    # Increment the TDMA cycle sequence
                    self.tdma_cycle_sequence += 1
                # If we haven't gotten all the priors yet, we need to wait
                elif self.priors_recieved <= self.num_agents-1:
                    self.tdma_cycle_sequence == 0
                else:
                    rospy.logwarn("TDMA Cycle Sequence Error")

            # If we are at the start of a new slot, log the status
            elif we_are_active != self.tdma_status.we_are_active:
                if we_are_active:
                    rospy.loginfo("[%s] TDMA Slot Started, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
                else:
                    rospy.loginfo("[%s] TDMA Slot Ended, Next Slot in %ssec" % (rospy.Time.now(), time_to_next_active))
                    # Reset the cycle status
                    self.cycle_status = 0
        # Regardless, update TDMA
        self.tdma_status = msg
        return
    
    # Ping Handling:
    def on_nmea_from_modem(self, msg: String):
        """This function receives NMEA messages from the modem
        """
        # Get the NMEA data type and the data
        nmea_msg = msg.data.split('*') #remove checksum
        nmea_msg = nmea_msg[0]
        nmea_string = msg.data.split(",") # split into fields
        nmea_type = nmea_string[0] # Get the NMEA string type
        data = nmea_string[1:] # Get the NMEA data only

        # Process the NMEA data by field
        if nmea_type == "$CAMPC" or nmea_type == "$CAMUC":
            self.ping_sent_time = rospy.Time.now()
            self.cycle_status +=1
        elif nmea_type == "$CAMPA" or nmea_type == "$CAMUA":
            self.acomms_event_time = rospy.Time.now()
            self.acomms_event.publish(self.acomms_event_time)
            self.on_ping_ack(nmea_type, data)
        elif nmea_type == "$CAMPR" or nmea_type == "$CAMUR":
            self.acomms_event_time = (rospy.Time.now() - self.ping_sent_time) / 2
            self.acomms_event.publish(self.acomms_event_time)
            self.cycle_status += 1
        else:
            return
        return    
    
    def on_preintegrated_imu(self, msg: PoseWithCovarianceStamped):
        """This function receives the preintegrated IMU pose from the estimator
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

        # Find the next empty BTWN key in partial graph data
        btwn_key = None
        for key in self.partial_graph_data.keys():
            if key.startswith("BTWN") and self.partial_graph_data[key]["time"] is None:
                btwn_key = key
                break

        if btwn_key is None:
            rospy.logwarn("No available BTWN slots in partial graph data structure!")
            return

        # Store the relative pose data
        self.partial_graph_data[btwn_key] = {
            "time": pose_time,
            "key1": key1,
            "key2": key2,
            "position": (position.x, position.y, position.z),
            "orientation": (orientation.x, orientation.y, orientation.z, orientation.w),
            "noise_sigmas": tuple(sigmas[:6])  # Extract position & orientation sigmas
        }
    
        rospy.loginfo(f"Stored relative pose in {btwn_key}: {self.partial_graph_data_structure[btwn_key]}")

        return

    def send_ping(self, target_addr):
        """This function sends a ping to the modem
        Args:
            target_addr (int): the target address
            symbol (str): the local key "A1" to put in the payload
        """
        symbol = chr(ord("A") + self.local_address) + str(self.modem_addresses[chr(ord("A") + self.local_address)][1]+1)
        # Set the ping request parameters
        ping_req = PingModemRequest()
        ping_req.dest = target_addr
        ping_req.rate = 0
        ping_req.cdr = 0
        ping_req.hex_data = self.pack_symbol_to_ping_payload(symbol)
        ping_req.timeout_sec = 5

        try:
            # send the ping and try to get a response
            rospy.loginfo("[%s] One Ping Only Vasily." % (rospy.Time.now()))
            ping_resp = self.ping_client(ping_req)

            if ping_resp.timed_out:
                rospy.logwarn("[%s] Ping to %s Timed Out" % (rospy.Time.now(), target_addr))
            else:
                rospy.loginfo("[%s] Ping to %s Successful" % (rospy.Time.now(), target_addr))
                src = ping_resp.cst.src
                dest = ping_resp.cst.dest
                owtt = ping_resp.one_way_travel_time
                measured_range = owtt * self.sound_speed
                timestamp = ping_resp.timestamp
                # Get the key association from the src and dest address
                src_symbol = chr(ord("A") + src)
                dest_chr = chr(ord("A") + dest)
                src_key = src_symbol + str(self.modem_addresses[src_symbol][1]+1)

                # Find the next empty RNG key in partial graph data
                rng_key = self.get_next_graph_key(self.partial_graph_data, "RNG")

                # Store the relative pose data
                self.partial_graph_data[rng_key] = {
                    "time": timestamp,
                    "key1": src_key,
                    "key2": dest_chr, 
                    "measured_range": measured_range,
                    "sigma_range": self.sigma_range,
                }
                
                rospy.loginfo(f"Stored relative pose in {rng_key}: {self.partial_graph_data[rng_key]}")

        except rospy.ServiceException as e:
            rospy.logerr("[%s] Ping Service Call Failed: %s" % (rospy.Time.now(), e))
        return    

    def on_ping_ack(self, nmea_type, data):
        """This function processes ping ack NMEA messages ($CAMUA or $CAMPA)
        Args:
            $CAMPA data (list): The NMEA data fields [SRC, DEST*CS]
            $CAMUA data (list): The NMEA data fields [SRC, DEST, HHHH*CS] (HHHH = hex payload, 4 bytes)
        """
        # Get the current time
        ack_time = rospy.Time.now()
        src = data[0]
        if nmea_type == "$CAMUA":
            dest = data[1]
            hex_payload = data[2].split("*")[0]
        else:
            dest = data[1].split("*")[0]
        # Convert the hex payload to a symbol
        remote_symbol = self.unpack_ping_payload_to_symbol(hex_payload)
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1]+1)
        time = rospy.Time.now()
        # Log the ping ack
        rospy.loginfo("[%s] Received Ping Ack from %s to %s, with symbol %s" % (ack_time, src, dest, remote_symbol))

        # Find the next empty RNG key in partial graph data
        rng_key = self.get_next_graph_key(self.partial_graph_data, "RNG")

        # Store the relative pose data
        self.partial_graph_data[rng_key] = {
            "time": time,
            "key1": local_symbol,
            "key2": remote_symbol, 
            "measured_range": None,
            "sigma_range": self.sigma_range,
        }
                
        rospy.loginfo(f"Stored relative pose in {rng_key}: {self.partial_graph_data[rng_key]}")
        return
    
    # Message Handling:
    def send_init_prior(self, dest):
        """This function sends a GPS prior to the estimator
        Args:
            position (Point): The GPS position
            dest (int): The destination address
        """
        # Generate a key for the GPS prior from the src address and 0
        local_symbol = chr(ord("A") + self.local_address) + str(self.modem_addresses[local_chr][1])
        dest_chr = chr(ord("A") + dest)
        # Get the current time
        time = rospy.Time.now()
        # Get the GPS position
        gps_position = self.initial_gps_fix
        gps_orientation = [0, 0, 0, 1]
        gps_noise = [1.7, 1.7, 4.3, 0.1, 0.1, 0.1]
        pos, ori, sig = self.pack_pose_prior_to_acomms_packet(gps_position, gps_orientation, gps_noise)
        # Create the GPS prior message
        gps_prior = PosePriorFactor()
        gps_prior.key = local_symbol
        gps_prior.pose.translation = pos
        gps_prior.pose.orientation = ori
        gps_prior.noise = sig
        # Publish the GPS prior
        self.init_prior_pubs[dest].publish(gps_prior)
        # Store the GPS prior as the next in the PRI section of the partial graph data
        pri_key = self.get_next_graph_key(self.partial_graph_data, "PRI")
        self.partial_graph_data[pri_key] = {
            "time": time,
            "key1": local_symbol,
            "position": pos,
            "orientation": ori,
            "noise_sigmas": sig
        }
        return

    def on_init_prior(self, msg: PosePriorFactor):
        """This function receives cycle init messages from the modem
        """
        # Cycle init is a [Symbol, Symbol, relative pose, noise, gps_pose, gps_noise]
        key = msg.key
        # Unpack the relative pose
        pos, ori, sig = self.unpack_acomms_packet_to_pose_prior(msg.pose.translation, msg.pose.orientation, msg.noise)
        # Get the current time
        time = rospy.Time.now()
        # Get the next available PRI key
        pri_key = self.get_next_graph_key(self.partial_graph_data, "PRI")
        # Add this to the partial graph data
        self.partial_graph_data[pri_key] = {
            "time": time,
            "key1": key,
            "position": pos,
            "orientation": ori,
            "noise_sigmas": sig
        }
        self.priors_recieved += 1
        return

    def check_partial_graph(self):
        """ Verifies that the partial graph is complete before its sent via acomms to other agents"""
        # Generate the expected number of BTWN, RNG, and PRI entries
        expected_rng = (self.num_agents-1)*2 + self.num_landmarks
        expected_btwn = expected_rng
        # Count the number of BTWN, RNG, and PRI entries
        num_btwn = 0
        num_rng = 0
        num_pri = 0
        for key in self.partial_graph_data.keys():
            if key.startswith("BTWN") and self.partial_graph_data[key]["time"] is not None:
                num_btwn += 1
            elif key.startswith("RNG") and self.partial_graph_data[key]["time"] is not None:
                num_rng += 1
            elif key.startswith("PRI") and self.partial_graph_data[key]["time"] is not None:
                num_pri += 1
        # Check if the partial graph is complete
        if num_btwn == expected_btwn and num_rng == expected_rng:
            rospy.loginfo("[%s] Partial Graph is Complete" % rospy.Time.now())
            # File the partial graph under the self.comms_cycle_data
            self.comms_cycle_data[self.tdma_cycle_sequence] = self.partial_graph_data.copy()
            return True
        else:
            # Note, this could occurr if a range failed ping we initiated failed or we didn't recieve one
            #TODO: Handle this failure more gracefully:
            # If we transmitted, but didn't get a reply:
            # - A between factor doesn't exist.
            # - A range factor doesn't exist and should not.
            # - The pose chain is not incremented
            # If we were pinged, but didn't get it:
            # - A between factor doesn't exist.
            # - A range factor doesn't exist and should not.
            # - The pose chain is not incremented
            # The system should allow this, but it has the potential to generate very large between factors.
            # The message compression currently allows pose deltas up to 12.7m and sigmas up to 25.5m, so it relies
            # on frequent ranges to keep them small.
            rospy.logwarn("[%s] Partial Graph is Incomplete" % rospy.Time.now())
            return False
    
    def send_partial_graph(self, dest):
        # Take the self.partial_graph_data and send it to a remote agent using the PartialGraph messag
        partial_graph_msg = PartialGraph()
        last_cycle = max(self.comms_cycle_data.keys())
        # For each row, construct the RelativePoseMin message
        for key in self.comms_cycle_data[last_cycle].keys():
            #common_sigmas = []
            num_poses = 0
            if key.startswith("BTWN") and self.comms_cycle_data[last_cycle][key]["time"] is not None:
                # Get the data for the BTWN row
                data = self.comms_cycle_data[last_cycle][key]
                # Get the key symbols
                key1 = data["key1"]
                key2 = data["key2"]
                # Get the relative pose data
                position = data["position"]
                orientation = data["orientation"]
                noise_sigmas = data["noise_sigmas"]
                # Create the RelativePoseMin message
                relpose_msg, common_sig = self.pack_relative_pose_to_acomms_packet(position, orientation, noise_sigmas)
                # Append the common sigmas (deprecated)
                #common_sigmas.append(common_sig)
                num_poses += 1
                # Add the relpose to the partial graph message
                if partial_graph_msg.rel_pose_0 is None:
                    base_index = int(key1[1:])
                    partial_graph_msg.rel_pose_0 = relpose_msg
                elif partial_graph_msg.rel_pose_1 is None:
                    partial_graph_msg.rel_pose_1 = relpose_msg
                elif partial_graph_msg.rel_pose_2 is None:
                    partial_graph_msg.rel_pose_2 = relpose_msg
                elif partial_graph_msg.rel_pose_3 is None:
                    partial_graph_msg.rel_pose_3 = relpose_msg
                elif partial_graph_msg.rel_pose_4 is None:
                    partial_graph_msg.rel_pose_4 = relpose_msg
                elif partial_graph_msg.rel_pose_5 is None:
                    partial_graph_msg.rel_pose_5 = relpose_msg
                else:
                    #NOTE: This should never happen, but if it did it would just ignore the later entries
                    rospy.logerr("[%s] Too many BTWN entries in partial graph" % rospy.Time.now())

            elif key.startswith("RNG") and self.comms_cycle_data[last_cycle][key]["time"] is not None and self.comms_cycle_data[last_cycle][key]["measured_range"] is not None:
                # Get the data for the RNG row
                data = self.comms_cycle_data[last_cycle]
                # Get the key symbols
                key1 = data["key1"]
                key2 = data["key2"]
                # Get the measured range
                measured_range = data["measured_range"]
                # Create the InitiatedRange message
                rng_msg = InitiatedRange()
                # Get the index of key1(the 1 of "A1") and subtract the full index to get the local index
                rng_msg.local_index = int(key1[1:])-base_index
                rng_msg.remote_addr = key2
                rng_msg.measured_range = int(measured_range * self.partial_graph_scale_factors["range_scale"])
                # Add the range to the partial graph message
                if partial_graph_msg.init_range_0 is None:
                    partial_graph_msg.init_range_0 = rng_msg
                elif partial_graph_msg.init_range_1 is None:
                    partial_graph_msg.init_range_1 = rng_msg
                elif partial_graph_msg.init_range_2 is None:
                    partial_graph_msg.init_range_2 = rng_msg
                elif partial_graph_msg.init_range_3 is None:
                    partial_graph_msg.init_range_3 = rng_msg

            elif key.startswith("RNG") and self.comms_cycle_data[last_cycle][key]["time"] is not None and self.comms_cycle_data[last_cycle][key]["measured_range"] is None:
                # Get the data for the RNG row
                data = self.comms_cycle_data[last_cycle]
                # Get the key symbols
                key1 = data["key1"]
                key2 = data["key2"]
                # Create the RecievedRange message
                rng_msg = RecievedRange()
                rng_msg.local_index = int(key1[1:])-base_index
                rng_msg.remote_addr = ord(key2[0]) - ord("A")
                rng_msg.remote_index = int(key2[1:])
                # Add the range to the partial graph message
                if partial_graph_msg.rcvd_range_0 is None:
                    partial_graph_msg.rcvd_range_0 = rng_msg
                elif partial_graph_msg.rcvd_range_1 is None:
                    partial_graph_msg.rcvd_range_1 = rng_msg
                else:
                    rospy.logerr("[%s] Too many RNG entries in partial graph" % rospy.Time.now())
            else:
                rospy.logerr("[%s] Unknown key in partial graph" % rospy.Time.now())

        # Assign the Key chain fields
        partial_graph_msg.local_address = self.local_address
        partial_graph_msg.full_index = base_index
        partial_graph_msg.num_poses = num_poses
        partial_graph_msg.sigma_range = self.sigma_range
        # Get the mean pitch and roll sigmas
        #partial_graph_msg.sigma_roll = np.mean([sig[3] for sig in common_sigmas])
        #partial_graph_msg.sigma_pitch = np.mean([sig[4] for sig in common_sigmas])
        # Publish the partial graph message
        self.partial_graph_pubs[dest].publish(partial_graph_msg)
        self.cycle_status += 1
        return

    def on_partial_graph(self, msg: PartialGraph):
        """This function receives partial graph messages from the modem (other agents)
        - UUpack these and append the equivalent BTWN, RNG entries to the most recent comms_cycle_data
        """
        # Get the most recent comms_cycle data
        last_cycle = max(self.comms_cycle_data.keys())
        # Get our local address
        local_address = self.local_address
        local_letter = chr(ord("A") + local_address)
        # Start by unpacking the partial graph message
        graph_address = msg.local_address
        full_index = msg.full_index
        num_poses = msg.num_poses
        # Generate a list of the keys in this message chr(local_address) + str(full_index)
        keys = [chr(graph_address) + str(full_index + i) for i in range(num_poses)]
        # Unpack the RelativePoseMin messages into a list
        rel_poses = [msg.rel_pose_0, msg.rel_pose_1, msg.rel_pose_2, msg.rel_pose_3, msg.rel_pose_4, msg.rel_pose_5]
        # Unpack the InitiatedRange messages into a list
        init_ranges = [msg.init_range_0, msg.init_range_1, msg.init_range_2, msg.init_range_3]
        # Unpack the RecievedRange messages into a list
        rcvd_ranges = [msg.rcvd_range_0, msg.rcvd_range_1]
        sigma_range = msg.sigma_range
        #sigma_roll = msg.sigma_roll
        #sigma_pitch = msg.sigma_pitch
        time = rospy.Time.now()
        # Check a few things about the existing data
        num_btwn = len([key for key in self.comms_cycle_data[last_cycle].keys() if key.startswith("BTWN")])
        num_rng = len([key for key in self.comms_cycle_data[last_cycle].keys() if key.startswith("RNG")])
        # Now for each rel_pose, unpack the values within, and append them to the partial graph data
        for i in range(num_poses):
            # Get the key for this pose
            key1 = keys[i]
            key2 = keys[i+1]
            # Unpack the relative pose
            pos, ori, sig = self.unpack_acomms_packet_to_relative_pose(rel_poses[i], (self.sigma_depth, self.sigma_roll, self.sigma_pitch))
            # Get the next available BTWN key
            btwn_key = f"BTWN_{num_btwn+1}"
            num_btwn += 1
            # Insert the relative pose into the partial graph data self.comms_cycle_data[self.tdma_cycle_sequence-1] at the btwn key
            self.comms_cycle_data[last_cycle][btwn_key] = {
                "time": time,
                "key1": key1,
                "key2": key2,
                "position": pos,
                "orientation": ori,
                "noise_sigmas": sig
            }

        for i in range(len(init_ranges)):
            # Get the key for this range
            key1 = keys[init_ranges[i].local_index]
            remote_letter = chr(ord("A") + init_ranges[i].remote_addr)
            measured_range = init_ranges[i].measured_range / self.partial_graph_scale_factors["range_scale"]
            # We need to check if key1 is in our existing data under the RNG section
            # If it is, we need to update the measured_range
            for key in self.comms_cycle_data[last_cycle].keys():
                if key.startswith("RNG") and (self.comms_cycle_data[last_cycle][key]["key1"] == key1 or self.comms_cycle_data[last_cycle][key]["key2"] == key1):
                    # We have an entry about this range already (it was initiated by us or by another agent), we just need to update the measured_range
                    self.comms_cycle_data[last_cycle][key]["measured_range"] = measured_range
                else:
                    # We don't have an entry about this already, we need to add it, although we don't know the key2
                    rng_key = f"RNG_{num_rng+1}"
                    num_rng += 1
                    self.comms_cycle_data[last_cycle][rng_key] = {
                        "time": time,
                        "key1": key1,
                        "key2": remote_letter,
                        "measured_range": measured_range,
                        "sigma_range": sigma_range,
                    }

        for i in range(len(rcvd_ranges)):
            # Get the key for this range
            key1 = keys[rcvd_ranges[i].local_index]
            key2 = chr(ord("A") + rcvd_ranges[i].remote_index) + str(rcvd_ranges[i].remote_index)
            # We need to chcek if we have a key2 in our existing data under the RNG section
            # If we do, we need to update the other key to the key1
            for key in self.comms_cycle_data[last_cycle].key():
                if key.startswith("RNG") and self.comms_cycle_data[last_cycle][key]["key1"] == key2:
                    self.comms_cycle_data[last_cycle][key]["key2"] = key1
                elif key.startswith("RNG") and self.comms_cycle_data[last_cycle][key]["key2"] == key2:
                    self.comms_cycle_data[last_cycle][key]["key1"] = key1
                else:
                    # We don't have an entry about this already, we need to add it
                    rng_key = f"RNG_{num_rng+1}"
                    num_rng += 1
                    self.comms_cycle_data[last_cycle][rng_key] = {
                        "time": time,
                        "key1": key1,
                        "key2": key2,
                        "measured_range": None,
                        "sigma_range": sigma_range,
                    }
        return
    
    # Cycle Management:
    def get_cycle_targets(self):
        """This function gets the target agent and target landmark for the cycle
        """
        if self.num_agents == 1:
            rospy.logerr("[%s] No agents to ping!" % rospy.Time.now())
        elif self.num_agents == 2 and not self.cycle_tgt_agent:
            # Review the modem_addresses and find the other agent
            for key, value in self.modem_addresses.items():
                # If Key is not the local address or a landmark (includes "L")
                if value[0] != self.local_address and not key.startswith("L"):
                    target_agent = value[0]
        elif self.num_agents == 3:
            # Review the modem_addresses and find the other agent
            for key, value in self.modem_addresses.items():
                # If Key is not the local address or a landmark (includes "L")
                if value[0] != self.local_address and not key.startswith("L") and not self.cycle_tgt_agent:
                    target_agent = value[0]
        elif self.num_agents > 3:
            rospy.logerr("[%s] More than 3 agents unsported!" % rospy.Time.now())
        else:
            rospy.logerr("[%s] At least 1 agent required!" % rospy.Time.now())

        if self.num_landmarks == 0:
            rospy.logerr("[%s] No landmarks to ping!" % rospy.Time.now())
        elif self.num_landmarks == 1 and not self.cycle_tgt_ldmk:
            # Review the modem_addresses and find the landmark
            for key, value in self.modem_addresses.items():
                # If Key is a landmark (includes "L")
                if key.startswith("L"):
                    target_landmark = value[0]
        elif self.num_landmarks == 2:
            # Review the modem_addresses and find the landmark
            for key, value in self.modem_addresses.items():
                # If Key is a landmark (includes "L")
                if key.startswith("L") and not self.cycle_tgt_ldmk:
                    target_landmark = value[0]
        elif self.num_landmarks > 2:
            rospy.logerr("[%s] More than 2 landmarks unsported!" % rospy.Time.now())
        else:
            pass

        self.cycle_tgt_agent = target_agent
        self.cycle_tgt_ldmk = target_landmark
        return target_agent, target_landmark
    
    def run(self):
        """The main loop for the cycle manager
        - Goal is to finish on_startup, then repeatedly cycle this function while the depth/time criterion is reached
        - should have an escape to send different messgaes if its the first attempt
        """
        # Get TDMA Status
        if self.tdma_status.we_are_active:
            if self.cycle_status == 0: # At cycle start
                # Set the target agent and target landmark
                target_agent, target_landmark = self.get_cycle_targets()
                # Send a ping to the target agent
                self.send_ping(target_agent)
            elif self.cycle_status == 1:
                # Check for ping timeout
                if (rospy.Time.now() - self.ping_sent_time).to_sec() > 5:
                    # Abort the ping, move on to the next step
                    self.cycle_status += 1
                else:
                    pass
            elif self.cycle_status == 2:
                # Send the partial graph to the target agent
                if self.tdma_cycle_sequence == 0:
                    # Send the initial priors
                    self.send_init_prior(target_agent)
                else:
                    # Send the partial graph
                    self.send_partial_graph(target_agent)
            elif self.cycle_status == 3:
                # Wait for 5sec for the partial graph to be sent
                if (rospy.Time.now() - self.acomms_event_time).to_sec() > 5:
                    # Assume the graph made it, move on to the next step
                    self.cycle_status += 1
            elif self.cycle_status == 4:
                # Send a ping to the target landmark
                self.send_ping(target_landmark)
            elif self.cycle_status == 5:
                # Check for ping timeout
                if (rospy.Time.now() - self.ping_sent_time).to_sec() > 5:
                    # Abort the ping, move on to the next step
                    self.cycle_status += 1
                else:
                    pass
            else:
                # We are at the end of the cycle do nothing
                pass
        else:
            # We are not active, do nothing
            pass   
        rospy.spin()
        return
    
    # Packing/Unpacking Functions:
    def pack_symbol_to_ping_payload(self, symbol):
        """This function converts a symbol to a hex payload
        Args:
            symbol (str): The symbol to convert
        Returns:
            str: The hex payload [0xXX, 0xXX, 0xXX, 0xXX]
        """
        # Get the symbol components
        letter = symbol[0].upper()
        number = int(symbol[1:])
        # Convert the letter to a 1-byte value (A = 1, B = 2, ..., Z = 26)
        letter_val = ord(letter) - ord('A') + 1
        # Convert the number to three bytes (little-endian format)
        num_bytes = number.to_bytes(3, 'little')  # 3 bytes for numbers up to 2^24-1
        packed_array = [letter_val, *num_bytes]
        assert len(packed_array) == 4
        hex_array = ["0x%02X" % val for val in packed_array]
        return hex_array

    def unpack_ping_payload_to_symbol(self, hex_array):
        """This function converts a hex payload to a symbol
        Args:
            hex_array (list): The hex payload
        Returns:
            str: The symbol
        """
        # Convert the hex array to a list of integers
        int_array = [int(val, 16) for val in hex_array]
        # Convert the letter to a character
        letter = chr(int_array[0] + ord('A') - 1)
        # Convert the number to an integer
        number = int.from_bytes(int_array[1:], 'little')
        # Return the symbol
        return letter + str(number)

    def pack_pose_prior_to_acomms_packet(self, position, orientation, sigmas):
        """Converts a relative pose to an acomms packet with dynamic scaling.
        
        Args:
            position (Point): The relative position (x, y, z).
            orientation (Quaternion): The relative orientation (x, y, z, w).
            sigmas (list): The standard deviations of the pose.

        Returns:
            tuple: Scaled integer representations of position, orientation, and sigmas.
            position must be scaled to an int16
            orientation must be scaled to an int16
            sigmas must be scaled to an int8
        """
        # Use the self.prior_scale_factors to scale the data, then convert to int without rounding
        pos_x = int(position.x * self.prior_scale_factors["x_scale"])
        pos_y = int(position.y * self.prior_scale_factors["y_scale"])
        pos_z = int(position.z * self.prior_scale_factors["z_scale"])
        ori_x = int(orientation.x * self.prior_scale_factors["qx_scale"])
        ori_y = int(orientation.y * self.prior_scale_factors["qy_scale"])
        ori_z = int(orientation.z * self.prior_scale_factors["qz_scale"])
        ori_w = int(orientation.w * self.prior_scale_factors["qw_scale"])
        sig_x = int(sigmas[0] * self.prior_scale_factors["sigma_x_scale"])
        sig_y = int(sigmas[1] * self.prior_scale_factors["sigma_y_scale"])
        sig_z = int(sigmas[2] * self.prior_scale_factors["sigma_z_scale"])
        sig_roll = int(sigmas[3] * self.prior_scale_factors["sigma_roll_scale"])
        sig_pitch = int(sigmas[4] * self.prior_scale_factors["sigma_pitch_scale"])
        sig_yaw = int(sigmas[5] * self.prior_scale_factors["sigma_yaw_scale"])
        # Check that all fit in the int8 or int16 bounds and provide a warning if they do not
        if not (-32768 <= pos_x <= 32767) or not (-32768 <= pos_y <= 32767) or not (-32768 <= pos_z <= 32767):
            rospy.logwarn("Position value exceeds int16 bounds")
        if not (-32768 <= ori_x <= 32767) or not (-32768 <= ori_y <= 32767) or not (-32768 <= ori_z <= 32767) or not (-32768 <= ori_w <= 32767):
            rospy.logwarn("Orientation value exceeds int16 bounds")
        if not (-128 <= sig_x <= 127) or not (-128 <= sig_y <= 127) or not (-128 <= sig_z <= 127) or not (-128 <= sig_roll <= 127) or not (-128 <= sig_pitch <= 127) or not (-128 <= sig_yaw <= 127):
            rospy.logwarn("Sigma value exceeds int8 bounds")
        # Return the scaled values
        return (pos_x, pos_y, pos_z), (ori_x, ori_y, ori_z, ori_w), (sig_x, sig_y, sig_z, sig_roll, sig_pitch, sig_yaw)

    def unpack_acomms_packet_to_pose_prior(self, position, orientation, sigmas):
        """Converts an acomms packet to a relative pose with dynamic scaling.
        
        Args:
            position (tuple): The scaled integer position (x, y, z).
            orientation (tuple): The scaled integer orientation (x, y, z, w).
            sigmas (tuple): The scaled integer standard deviations of the pose.

        Returns:
            tuple: The relative position, orientation, and sigmas.
        """
        # Undo the scaling perfomed by pack_pose_prior_to_acomms_packet
        pos_x = position[0] / self.prior_scale_factors["x_scale"]
        pos_y = position[1] / self.prior_scale_factors["y_scale"]
        pos_z = position[2] / self.prior_scale_factors["z_scale"]
        ori_x = orientation[0] / self.prior_scale_factors["qx_scale"]
        ori_y = orientation[1] / self.prior_scale_factors["qy_scale"]
        ori_z = orientation[2] / self.prior_scale_factors["qz_scale"]
        ori_w = orientation[3] / self.prior_scale_factors["qw_scale"]
        sig_x = sigmas[0] / self.prior_scale_factors["sigma_x_scale"]
        sig_y = sigmas[1] / self.prior_scale_factors["sigma_y_scale"]
        sig_z = sigmas[2] / self.prior_scale_factors["sigma_z_scale"]
        sig_roll = sigmas[3] / self.prior_scale_factors["sigma_roll_scale"]
        sig_pitch = sigmas[4] / self.prior_scale_factors["sigma_pitch_scale"]
        sig_yaw = sigmas[5] / self.prior_scale_factors["sigma_yaw_scale"]
        # Return the unscaled values
        return (pos_x, pos_y, pos_z), (ori_x, ori_y, ori_z, ori_w), (sig_x, sig_y, sig_z, sig_roll, sig_pitch, sig_yaw)
    
    def pack_relative_pose_to_acomms_packet(self, position, orientation, sigmas):
        """ Pack the relative pose in acomms packet format
        Args:
            position (Point): The relative position
            orientation (Quaternion): The relative orientation
            sigmas (list): The standard deviations of the pose
            
        Returns:
            tuple: Scaled integer representations of position, orientation, and sigmas.
            position must be scaled to an int8
            orientation must be scaled to an int8
            sigmas must be scaled to an uint8"""
        # Use the self.partial_graph_scale_factors to scale the data, then convert to int without rounding
        pos_x = int(position.x * self.partial_graph_scale_factors["dx_scale"])
        pos_y = int(position.y * self.partial_graph_scale_factors["dy_scale"])
        pos_z = int(position.z * self.partial_graph_scale_factors["dz_scale"])
        ori_x = int(orientation.x * self.partial_graph_scale_factors["dqx_scale"])
        ori_y = int(orientation.y * self.partial_graph_scale_factors["dqy_scale"])
        ori_z = int(orientation.z * self.partial_graph_scale_factors["dqz_scale"])
        ori_w = int(orientation.w * self.partial_graph_scale_factors["dqw_scale"])
        sig_x = int(sigmas[0] * self.partial_graph_scale_factors["sigma_x_scale"])
        sig_y = int(sigmas[1] * self.partial_graph_scale_factors["sigma_y_scale"])
        sig_z = int(sigmas[2] * self.partial_graph_scale_factors["sigma_z_scale"])
        sig_roll = int(sigmas[3] * self.partial_graph_scale_factors["sigma_roll_scale"])
        sig_pitch = int(sigmas[4] * self.partial_graph_scale_factors["sigma_pitch_scale"])
        sig_yaw = int(sigmas[5] * self.partial_graph_scale_factors["sigma_yaw_scale"])
        # Check that all fit in the int8 or int16 bounds and provide a warning if they do not
        if not (-128 <= pos_x <= 127) or not (-128 <= pos_y <= 127) or not (-128 <= pos_z <= 127):
            rospy.logwarn("Position value exceeds int8 bounds")
        if not (-128 <= ori_x <= 127) or not (-128 <= ori_y <= 127) or not (-128 <= ori_z <= 127) or not (-128 <= ori_w <= 127):
            rospy.logwarn("Orientation value exceeds int8 bounds")
        if not (0 <= sig_x <= 255) or not (0 <= sig_y <= 255) or not (0 <= sig_z <= 255) or not (0 <= sig_roll <= 255) or not (0 <= sig_pitch <= 255) or not (0 <= sig_yaw <= 255):
            rospy.logwarn("Sigma value exceeds uint8 bounds")
        # Build a relative pose msg
        rel_pose = RelativePoseMin()
        rel_pose.x = pos_x
        rel_pose.y = pos_y
        rel_pose.z = pos_z
        rel_pose.qx = ori_x
        rel_pose.qy = ori_y
        rel_pose.qz = ori_z
        rel_pose.qw = ori_w
        rel_pose.sigma_x = sig_x
        rel_pose.sigma_y = sig_y
        rel_pose.sigma_yaw = sig_yaw
        # Return the message, plus the packed common sigmas
        return rel_pose, (sigmas[2], sigmas[3], sigmas[4])
    
    def unpack_acomms_packet_to_relative_pose(self, msg:RelativePoseMin, common_sigmas):
        """ Unpack the relative pose in acomms packet format
        Args:
            msg (RelativePoseMin): The relative pose message
            common_sigmas (tuple): The common sigmas for the pose [sigma_depth, sigma_roll, sigma_pitch]
        Returns:
            tuple: The relative position, orientation, and sigmas.
        """
        # Undo the scaling perfomed by pack_relative_pose_to_acomms_packet
        pos_x = msg.x / self.partial_graph_scale_factors["dx_scale"]
        pos_y = msg.y / self.partial_graph_scale_factors["dy_scale"]
        pos_z = msg.z / self.partial_graph_scale_factors["dz_scale"]
        ori_x = msg.qx / self.partial_graph_scale_factors["dqx_scale"]
        ori_y = msg.qy / self.partial_graph_scale_factors["dqy_scale"]
        ori_z = msg.qz / self.partial_graph_scale_factors["dqz_scale"]
        ori_w = msg.qw / self.partial_graph_scale_factors["dqw_scale"]
        sig_x = msg.sigma_x / self.partial_graph_scale_factors["sigma_x_scale"]
        sig_y = msg.sigma_y / self.partial_graph_scale_factors["sigma_y_scale"]
        sig_z = common_sigmas[0]
        sig_roll = common_sigmas[1]
        sig_pitch = common_sigmas[2]
        sig_yaw = msg.sigma_yaw / self.partial_graph_scale_factors["sigma_yaw_scale"]
        # Return the unscaled values
        return (pos_x, pos_y, pos_z), (ori_x, ori_y, ori_z, ori_w), (sig_x, sig_y, sig_z, sig_roll, sig_pitch, sig_yaw)
        
def main():
    manager = CycleManager()
    manager.on_startup()
    while not rospy.is_shutdown():
        manager.run()

if __name__ == '__main__':
    main()