#!/usr/bin/env python3
import rospy
import numpy as np
from datetime import datetime
from ros_acomms_msgs.msg import(
    TdmaStatus
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)
from spurdog_acomms.msg import(
    InitPrior, PartialGraph
)
from std_msgs.msg import String, Time

class CycleTest:
    """ This is a node to test the comms capability dockside
    It will continuously iterate the comms cycle between an agent and some landmarks
    It won't actually pass data, just template packets.
    """
    def __init__(self):
        rospy.init_node('comms_cycle_test', anonymous=True)
        self.local_address = rospy.get_param("~modem_address", 0)
        self.num_agents = rospy.get_param("~num_agents", 1)
        self.num_landmarks = rospy.get_param("~num_landmarks", 0)
        self.landmarks = rospy.get_param("~landmarks", {}) # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = rospy.get_param("~sound_speed", 1500)
        self.sigma_range = rospy.get_param("~sigma_range", 1)
        self.sigma_depth = rospy.get_param("~sigma_depth", 1)
        self.sigma_roll = rospy.get_param("~sigma_roll", 0.1)
        self.sigma_pitch = rospy.get_param("~sigma_pitch", 0.1)
        self.tdma_status = TdmaStatus()
        self.modem_addresses = {}
        self.ping_sent_time = None
        self.acomms_event_time = None
        self.cycle_status = True
        self.tdma_cycle_sequence = 0
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
        self.init_prior_pubs = {}
        self.init_prior_subs = {}
        self.partial_graph_pubs = {}
        self.partial_graph_subs = {}
        self.cycle_target_mapping = {}
        # Check services, then initialize them
        rospy.wait_for_service("modem/ping_modem")
        self.ping_client = rospy.ServiceProxy("modem/ping_modem", PingModem)
        # Monitor modem TDMA status to control the comms cycle
        self.tdma_from_modem = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_status)
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        self.nmea_to_modem = rospy.Publisher("modem/nmea_to_modem", String, queue_size=10)
        # Initialize the pubs/subs for create relative pose measurements from the estimator
        self.acomms_event = rospy.Publisher("acomms_event", Time, queue_size=1)

    # Setup Functions:
    def setup_addresses(self):
        """This function sets up the number of agents and landmarks
        """
        # Build the modem addresses
        if self.num_agents == 1 and self.num_landmarks == 0:
            rospy.logerr("[%s] No one to ping!" % rospy.Time.now())
        else:
            # Build the key tracker, assigning a unique integer to each agent and landmark
            for i in range(self.num_agents):
                letter = chr(ord('A') + i)
                if i == self.local_address:
                    # Assigns an integer for indexing our own poses
                    self.modem_addresses[letter] = [i, 0]
                else:
                    self.modem_addresses[letter] = [i]
                last_assigned = i
            last_assigned += 1
            for i in range(self.num_landmarks):
                last_assigned += i
                self.modem_addresses["L%d" % i] = [last_assigned,0]
        rospy.loginfo("[%s] Modem Addresses: %s" % (rospy.Time.now(), self.modem_addresses))
        return

    def setup_cycle_targets(self):
        """This function sets up the cycle targets for the agents and landmarks."""

        # Validate num_agents range
        if not (1 <= self.num_agents <= 3):
            rospy.logerr("[%s] Invalid number of agents: %d" % (rospy.Time.now(), self.num_agents))
            return

        # Validate num_landmarks range
        if not (0 <= self.num_landmarks <= 2):
            rospy.logerr("[%s] Invalid number of landmarks: %d" % (rospy.Time.now(), self.num_landmarks))
            return

       # Determine TDMA slots based on number of agents
        expected_slots = 6
        if self.num_agents == 3:
            active_slot_1 = self.local_address
            active_slot_2 = (active_slot_1 + expected_slots // 2) % expected_slots
            active_slots = [active_slot_1, active_slot_2]
        elif self.num_agents == 2:
            # We have 6 slots, so agent 0 and agent 1 will alternate
            active_slots = [0, 2, 4]
            active_slots = [(i + self.local_address) % expected_slots for i in active_slots]
        elif self.num_agents == 1:
            active_slots = [0,1,2,3,4,5]
        else:
            rospy.logerr("[%s] Invalid number of agents: %d" % (rospy.Time.now(), self.num_agents))
            return
        # Log the active slots
        rospy.loginfo("[%s] Active Slots: %s" % (rospy.Time.now(), active_slots))

        # Build a list of the target agents addresses from modem_addresses
        # Exclude the local address from the target agents
        tgt_agents = [self.modem_addresses[chr(ord('A') + i)][0] for i in range(self.num_agents) if i != self.local_address]
        rospy.loginfo("[%s] Target Agents: %s" % (rospy.Time.now(), tgt_agents))
        # Build a list of target landmark
        tgt_ldmks = [self.modem_addresses["L%d" % i][0] for i in range(self.num_landmarks)]
        rospy.loginfo("[%s] Target Landmarks: %s" % (rospy.Time.now(), tgt_ldmks))
        # Generate cycle target mapping
        for i in range(expected_slots):
            if i in active_slots:
                # Ensure we don't attempt modulo with empty lists
                agent_target = tgt_agents[i % len(tgt_agents)] if tgt_agents else None
                ldmk_target = tgt_ldmks[i % len(tgt_ldmks)] if tgt_ldmks else None
                self.cycle_target_mapping[i] = [agent_target, ldmk_target]
            else:
                self.cycle_target_mapping[i] = [[],[]]

        rospy.loginfo("[%s] Cycle Target Mapping: %s" % (rospy.Time.now(), self.cycle_target_mapping))
        return

    def on_startup(self):
        """This function initializes the modem addresses and sets up the initial relative pose measurements
        """
        self.setup_addresses()
        self.setup_cycle_targets()
        # Initialize the subs/pubs for the test messages
        # Note: we only publish to the modem addresses that are not our own
        for key, value in self.modem_addresses.items():
            # If the key contains 'L', it is a landmark and we don't need to publish to it
            if "L" in key:
                continue
            elif value[0] != self.local_address:
                # Create the publisher for the initial prior factor
                init_prior_pub = rospy.Publisher("modem/init_prior_%s" % value[0], InitPrior, queue_size=1)
                self.init_prior_pubs[key] = init_prior_pub
                # Create the subscriber for the initial prior factor
                init_prior_sub = rospy.Subscriber("modem/init_prior_%s" % value[0], InitPrior, self.on_init_prior)
                self.init_prior_subs[key] = init_prior_sub
                # Create the subscriber for the partial graph
                partial_graph_sub = rospy.Subscriber("modem/partial_graph_%s" % value[0], PartialGraph, self.on_partial_graph)
                self.partial_graph_subs[key] = partial_graph_sub
                # Create the publisher for the partial graph
                partial_graph_pub = rospy.Publisher("modem/partial_graph_%s" % value[0], PartialGraph, queue_size=1)
                self.partial_graph_pubs[key] = partial_graph_pub
            else:
                # If the key is our own address, we don't need to subscribe or publish to it
                rospy.loginfo("[%s] Skipping %s, it's our own address" % (rospy.Time.now(), key))
                continue
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

        # Check if the TDMA slot has changed:
        if current_slot != self.tdma_status.current_slot:
            if current_slot == 0:
                self.tdma_cycle_sequence += 1
            # If we are at the start of a new slot, log the status
            if we_are_active == True and self.tdma_status.we_are_active == False:
                self.cycle_status = True
                rospy.loginfo("[%s] TDMA Slot Started, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
            else:
                rospy.loginfo("[%s] TDMA Slot Ended, Next Slot in %ssec" % (rospy.Time.now(), time_to_next_active))
        # Regardless, update TDMA
        self.tdma_status.current_slot = current_slot
        self.tdma_status.we_are_active = we_are_active
        self.tdma_status.remaining_slot_seconds = remaining_sec_in_slot
        self.tdma_status.remaining_active_seconds = remaining_active_sec
        self.tdma_status.time_to_next_active = time_to_next_active
        self.tdma_status.slot_duration_seconds = slot_duration
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
        if nmea_type == "$CACMD":
            self.ping_sent_time = rospy.Time.now()
        elif nmea_type == "$CACMA":
            acomms_event_time = rospy.Time.from_sec(datetime.strptime(data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp())
            src = data[2]
            dest = data[3]
            if dest != self.local_address:
                rospy.loginfo("[%s] Overheard Ping from %s to %s" % (acomms_event_time, src, dest))
            else:
                self.acomms_event.publish(acomms_event_time)
                self.modem_addresses[chr(ord("A") + self.local_address)][1] += 1
                #self.on_ping_ack(nmea_type, data)
        elif nmea_type == "$CACMR":
            # TODO: Fix this once you observe one
            acomms_event_time = (rospy.Time.now() - self.ping_sent_time) / 2
            self.acomms_event.publish(acomms_event_time)
            self.modem_addresses[chr(ord("A") + self.local_address)][1] += 1
            rospy.loginfo("[%s] Received Ping Response from %s" % (rospy.Time.now(), data[0]))
        elif nmea_type == "$CARFP" and data[5] == "-1":
            #rospy.loginfo("[%s] Received CARFP with length %s" % (rospy.Time.now(), len(data)))
            time, src, dest, payload = self.extract_ping_data_from_carfp(data)
            rospy.loginfo("[%s] Received CARFP from %s to %s with payload %s" % (time, src, dest, payload))
            if dest != self.local_address:
                rospy.loginfo("[%s] Overheard Ping from %s to %s" % (time, src, dest))
            else:
                self.acomms_event.publish(time)
                self.modem_addresses[chr(ord("A") + self.local_address)][1] += 1
                rospy.loginfo("[%s] Received Ping from %s to %s with payload %s" % (time, src, dest, payload))
        else:
            return
        return

    def send_ping(self, target_addr):
        """This function sends a ping to the modem
        Args:
            target_addr (int): the target address
            symbol (str): the local key "A1" to put in the payload
        """
        # Get the symbol for the ping payload and then increment the local address
        symbol = chr(ord("A") + self.local_address) + str(self.modem_addresses[chr(ord("A") + self.local_address)][1]+1)
        # Set the ping request parameters
        ping_req = PingModemRequest()
        ping_req.dest = target_addr
        ping_req.rate = 1
        ping_req.cdr = 0
        #ping_req.hexdata = bytearray([0,1,0,0])  # Placeholder for the payload
        # This works to pass it to a codec, but the decoding step fails
        #ping_req.hexdata = bytearray([172]) + bytearray([1]) + bytearray(symbol, "utf-8")
        ping_req.hexdata = bytearray(symbol.encode("utf-8"))
        ping_req.timeout_sec = 5

        try:
            # send the ping and try to get a response
            rospy.loginfo("[%s] One Ping Only Vasily." % (rospy.Time.now()))
            ping_resp = self.ping_client(ping_req)
        # ping_nmea = "$CCMPC,%s,%s" % (self.local_address, target_addr)
        # self.nmea_to_modem.publish(ping_nmea)
        # Wait 5sec
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
        if nmea_type == "$CAMPA":
            dest = data[1].split("*")[0]
        else:
            dest = data[1]
            hex_payload = data[2].split("*")[0]
            remote_symbol = hex_payload.decode("utf-8")
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1]+1)
        time = rospy.Time.now()
        # Log the ping ack
        rospy.loginfo("[%s] Received Ping Ack from %s to %s, with symbol %s" % (ack_time, src, dest, remote_symbol))

        return
        # Packing/Unpacking Functions:

    def extract_ping_data_from_carfp(self, data):
        """This function extracts the ping data from a CARFP NMEA message
        Args:
            data (list): The NMEA data fields [HHHH, SRC, DEST, PAYLOAD]
        Returns:
            tuple: (time, src, dest, hex_payload)
        """
        time = rospy.Time.from_sec(datetime.strptime(data[0],"%Y-%m-%dT%H:%M:%S.%f").timestamp())
        src = int(data[2])
        dest = int(data[3])
        hex_data = data[9].split(";")[-1]  # Remove checksum if present
        hex_payload = bytearray.fromhex(hex_data)
        # Convert hex payload to string if needed
        payload = hex_payload.decode("utf-8") if hex_payload else ""
        return time, src, dest, payload

    # Generate Test Messages:
    def generate_random_quaternion(self, ):
        """This function generates a random quaternion
        Returns:
            np.array: A random quaternion
        """
        u1, u2, u3 = np.random.rand(0, 1, size=3)
        qx = np.sqrt(1 - u1) * np.sin(2 * np.pi * u2)
        qy = np.sqrt(1 - u1) * np.cos(2 * np.pi * u2)
        qz = np.sqrt(u1) * np.sin(2 * np.pi * u3)
        qw = np.sqrt(u1) * np.cos(2 * np.pi * u3)
        # Normalize
        norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        if norm > 0:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm
        else:
            rospy.logerr("[%s] Generated quaternion has zero norm!" % rospy.Time.now())
        return np.array([qx, qy, qz, qw])

    def generate_test_init_prior_data(self, method: str):
        """This function generates a test initial prior factor message
        Returns:
            PosePriorFactor: The initial prior factor message
        """
        if method == "zero":
            # Build an array of initial position
            initial_position = np.array([0, 0, 0])  # Placeholder for the initial position
            # Build an array of initial orientation (quaternion)
            initial_orientation = np.array([0, 0, 0, 1])  # Placeholder for the initial orientation
            # Build an array of initial sigmas (covariance)
            initial_sigmas = np.array([1,1,1,1,1,1])  # Placeholder for the initial sigmas
        elif method == "random":
            # Generate random initial position within reasonable bounds
            initial_x = np.random.rand(-3000, 1500)
            initial_y = np.random.rand(-3000, 500)
            initial_z = np.random.rand(-5, 10)
            initial_position = np.array([initial_x, initial_y, initial_z])
            # Generate a random initial orientation (quaternion)
            initial_orientation = self.generate_random_quaternion()
            # Generate random initial sigmas (covariance)
            initial_sigmas = np.random.rand(0, 100, size = 6)  # Placeholder for the initial sigmas
        else:
            rospy.logerr("[%s] Unknown method for generating initial prior factor!" % rospy.Time.now())
            return None
        return initial_position, initial_orientation, initial_sigmas

    def generate_test_partial_graph_data(self, method: str):
        """This function generates a test partial graph message
        Returns:
            PartialGraph: The partial graph message
        """
        msg = PartialGraph()
        msg.local_addr = int(self.local_address)
        msg.full_index = int(self.modem_addresses[chr(ord("A") + self.local_address)][1])
        msg.num_poses = 6  # Placeholder for the number of poses
        # Build an array of local indices
        msg.local_index_0 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1])
        msg.local_index_1 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 1
        msg.local_index_2 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 2
        msg.local_index_3 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 3
        msg.local_index_4 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 4
        msg.local_index_5 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 5
        # Build an array of remote addresses
        msg.remote_addr_0 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1])
        msg.remote_addr_1 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 1
        msg.remote_addr_2 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 2
        msg.remote_addr_3 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 3
        msg.remote_addr_4 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 4
        msg.remote_addr_5 = int(self.modem_addresses[chr(ord("A") + self.local_address)][1]) + 5

        if method == "zero":
            # Build an array of relative positions
            msg.relative_pos_0 = np.array([0, 0, 0])
            msg.relative_pos_1 = np.array([0, 0, 0])
            msg.relative_pos_2 = np.array([0, 0, 0])
            msg.relative_pos_3 = np.array([0, 0, 0])
            msg.relative_pos_4 = np.array([0, 0, 0])
            msg.relative_pos_5 = np.array([0, 0, 0])
            # Build an array of relative orientations (quaternion)
            msg.relative_rot_0 = np.array([0, 0, 0, 1])
            msg.relative_rot_1 = np.array([0, 0, 0, 1])
            msg.relative_rot_2 = np.array([0, 0, 0, 1])
            msg.relative_rot_3 = np.array([0, 0, 0, 1])
            msg.relative_rot_4 = np.array([0, 0, 0, 1])
            msg.relative_rot_5 = np.array([0, 0, 0, 1])
            # Build an array of unique sigmas (covariance)
            msg.unique_sigmas_0 = np.array([1, 1, 1])
            msg.unique_sigmas_1 = np.array([1, 1, 1])
            msg.unique_sigmas_2 = np.array([1, 1, 1])
            msg.unique_sigmas_3 = np.array([1, 1, 1])
            msg.unique_sigmas_4 = np.array([1, 1, 1])
            msg.unique_sigmas_5 = np.array([1, 1, 1])
            # Build an array of measured ranges
            msg.meas_range_0 = np.array([1])
            msg.meas_range_1 = np.array([1])
            msg.meas_range_2 = np.array([1])
            msg.meas_range_3 = np.array([1])

        elif method == "random":
            # Generate random relative positions within reasonable bounds
            msg.relative_pos_0 = np.array([np.random(-327, 327), np.random(-327, 327), np.random(-15, 15)])
            msg.relative_pos_1 = np.array([np.random(-12.7, 12.7), np.random(-12.7, 12.7), np.random(-12.7, 12.7)])
            msg.relative_pos_2 = np.array([np.random(-12.7, 12.7), np.random(-12.7, 12.7), np.random(-12.7, 12.7)])
            msg.relative_pos_3 = np.array([np.random(-12.7, 12.7), np.random(-12.7, 12.7), np.random(-12.7, 12.7)])
            msg.relative_pos_4 = np.array([np.random(-12.7, 12.7), np.random(-12.7, 12.7), np.random(-12.7, 12.7)])
            msg.relative_pos_5 = np.array([np.random(-12.7, 12.7), np.random(-12.7, 12.7), np.random(-12.7, 12.7)])
            # Generate random relative orientations (quaternion)
            msg.relative_rot_0 = self.generate_random_quaternion()
            msg.relative_rot_1 = self.generate_random_quaternion()
            msg.relative_rot_2 = self.generate_random_quaternion()
            msg.relative_rot_3 = self.generate_random_quaternion()
            msg.relative_rot_4 = self.generate_random_quaternion()
            msg.relative_rot_5 = self.generate_random_quaternion()
            # Generate random unique sigmas (covariance)
            msg.unique_sigmas_0 = np.array([np.random(0, 25.5), np.random(0, 25.5), np.random(0, 6.28)])
            msg.unique_sigmas_1 = np.array([np.random(0, 25.5), np.random(0, 25.5), np.random(0, 6.28)])
            msg.unique_sigmas_2 = np.array([np.random(0, 25.5), np.random(0, 25.5), np.random(0, 6.28)])
            msg.unique_sigmas_3 = np.array([np.random(0, 25.5), np.random(0, 25.5), np.random(0, 6.28)])
            msg.unique_sigmas_4 = np.array([np.random(0, 25.5), np.random(0, 25.5), np.random(0, 6.28)])
            msg.unique_sigmas_5 = np.array([np.random(0, 25.5), np.random(0, 25.5), np.random(0, 6.28)])
            # Generate random measured ranges
            msg.meas_range_0 = np.random.rand(0, 2000)
            msg.meas_range_1 = np.random.rand(0, 2000)
            msg.meas_range_2 = np.random.rand(0, 2000)
            msg.meas_range_3 = np.random.rand(0, 2000)

        else:
            rospy.logerr("[%s] Unknown method for generating partial graph!" % rospy.Time.now())
            return None
        return msg

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

    def encode_partial_graph_data_as_int(self, partial_graph: PartialGraph):
        """This function encodes the partial graph data into a message
        Args:
            partial_graph (PartialGraph): The partial graph message
        Returns:
            PartialGraph: The encoded partial graph message
        """
        encoded_msg = PartialGraph()
        encoded_msg.local_addr = int(partial_graph.local_addr)
        encoded_msg.full_index = int(partial_graph.full_index)
        encoded_msg.num_poses = int(partial_graph.num_poses)
        # Encode the local indices
        encoded_msg.local_index_0 = int(partial_graph.local_index_0)
        encoded_msg.local_index_1 = int(partial_graph.local_index_1)
        encoded_msg.local_index_2 = int(partial_graph.local_index_2)
        encoded_msg.local_index_3 = int(partial_graph.local_index_3)
        encoded_msg.local_index_4 = int(partial_graph.local_index_4)
        encoded_msg.local_index_5 = int(partial_graph.local_index_5)
        # Encode the remote addresses
        encoded_msg.remote_addr_0 = int(partial_graph.remote_addr_0)
        encoded_msg.remote_addr_1 = int(partial_graph.remote_addr_1)
        encoded_msg.remote_addr_2 = int(partial_graph.remote_addr_2)
        encoded_msg.remote_addr_3 = int(partial_graph.remote_addr_3)
        encoded_msg.remote_addr_4 = int(partial_graph.remote_addr_4)
        encoded_msg.remote_addr_5 = int(partial_graph.remote_addr_5)
        # Verify all the addresses are within uint8
        if not (np.all(np.abs([encoded_msg.local_addr, encoded_msg.full_index,
                             encoded_msg.local_index_0, encoded_msg.local_index_1,
                             encoded_msg.local_index_2, encoded_msg.local_index_3,
                             encoded_msg.local_index_4, encoded_msg.local_index_5,
                             encoded_msg.remote_addr_0, encoded_msg.remote_addr_1,
                             encoded_msg.remote_addr_2, encoded_msg.remote_addr_3,
                             encoded_msg.remote_addr_4, encoded_msg.remote_addr_5]) <= 255)):
            rospy.logerr("[%s] Partial graph addresses out of range!" % rospy.Time.now())
            return None
        # Verify the number of poses is within uint8
        if not (0 <= encoded_msg.num_poses <= 6):
            rospy.logerr("[%s] Number of poses out of range!" % rospy.Time.now())
            return None
        # Encode the relative positions
        #TODO: See if you can enforce the dtype like you can in numpy
        # relative_pos_0 is a int16[3], multiply by scale, then convert to int16
        encoded_msg.relative_pos_0 = list([
            int(partial_graph.relative_pos_0[0] * self.codec_scale_factors["partial_graph"]["x_0"]),
            int(partial_graph.relative_pos_0[1] * self.codec_scale_factors["partial_graph"]["y_0"]),
            int(partial_graph.relative_pos_0[2] * self.codec_scale_factors["partial_graph"]["z_0"])
        ])
        # relative_pos_1-5 are int8[3], multiply by scale, then convert to int8
        encoded_msg.relative_pos_1 = list([
            int(partial_graph.relative_pos_1[0] * self.codec_scale_factors["partial_graph"]["x"]),
            int(partial_graph.relative_pos_1[1] * self.codec_scale_factors["partial_graph"]["y"]),
            int(partial_graph.relative_pos_1[2] * self.codec_scale_factors["partial_graph"]["z"])
        ])
        encoded_msg.relative_pos_2 = list([
            int(partial_graph.relative_pos_2[0] * self.codec_scale_factors["partial_graph"]["x"]),
            int(partial_graph.relative_pos_2[1] * self.codec_scale_factors["partial_graph"]["y"]),
            int(partial_graph.relative_pos_2[2] * self.codec_scale_factors["partial_graph"]["z"])
        ])
        encoded_msg.relative_pos_3 = list([
            int(partial_graph.relative_pos_3[0] * self.codec_scale_factors["partial_graph"]["x"]),
            int(partial_graph.relative_pos_3[1] * self.codec_scale_factors["partial_graph"]["y"]),
            int(partial_graph.relative_pos_3[2] * self.codec_scale_factors["partial_graph"]["z"])
        ])
        encoded_msg.relative_pos_4 = list([
            int(partial_graph.relative_pos_4[0] * self.codec_scale_factors["partial_graph"]["x"]),
            int(partial_graph.relative_pos_4[1] * self.codec_scale_factors["partial_graph"]["y"]),
            int(partial_graph.relative_pos_4[2] * self.codec_scale_factors["partial_graph"]["z"])
        ])
        encoded_msg.relative_pos_5 = list([
            int(partial_graph.relative_pos_5[0] * self.codec_scale_factors["partial_graph"]["x"]),
            int(partial_graph.relative_pos_5[1] * self.codec_scale_factors["partial_graph"]["y"]),
            int(partial_graph.relative_pos_5[2] * self.codec_scale_factors["partial_graph"]["z"])
        ])
        # Verify they are all in the correct range
        if not (np.all(np.abs(encoded_msg.relative_pos_0) <= 32767) and
                np.all(np.abs(encoded_msg.relative_pos_1) <= 127) and
                np.all(np.abs(encoded_msg.relative_pos_2) <= 127) and
                np.all(np.abs(encoded_msg.relative_pos_3) <= 127) and
                np.all(np.abs(encoded_msg.relative_pos_4) <= 127) and
                np.all(np.abs(encoded_msg.relative_pos_5) <= 127)):
            rospy.logerr("[%s] Partial graph relative positions out of range!" % rospy.Time.now())
            return None
        # Encode the relative rotations (quaternion)
        # relative_rot_0-5 is a int8[4] (quaternion), multiply by scale, then convert to int8
        encoded_msg.relative_rot_0 = list([
            int(partial_graph.relative_rot_0[0] * self.codec_scale_factors["partial_graph"]["qx"]),
            int(partial_graph.relative_rot_0[1] * self.codec_scale_factors["partial_graph"]["qy"]),
            int(partial_graph.relative_rot_0[2] * self.codec_scale_factors["partial_graph"]["qz"]),
            int(partial_graph.relative_rot_0[3] * self.codec_scale_factors["partial_graph"]["qw"])
        ])
        encoded_msg.relative_rot_1 = list([
            int(partial_graph.relative_rot_1[0] * self.codec_scale_factors["partial_graph"]["qx"]),
            int(partial_graph.relative_rot_1[1] * self.codec_scale_factors["partial_graph"]["qy"]),
            int(partial_graph.relative_rot_1[2] * self.codec_scale_factors["partial_graph"]["qz"]),
            int(partial_graph.relative_rot_1[3] * self.codec_scale_factors["partial_graph"]["qw"])
        ])
        encoded_msg.relative_rot_2 = list([
            int(partial_graph.relative_rot_2[0] * self.codec_scale_factors["partial_graph"]["qx"]),
            int(partial_graph.relative_rot_2[1] * self.codec_scale_factors["partial_graph"]["qy"]),
            int(partial_graph.relative_rot_2[2] * self.codec_scale_factors["partial_graph"]["qz"]),
            int(partial_graph.relative_rot_2[3] * self.codec_scale_factors["partial_graph"]["qw"])
        ])
        encoded_msg.relative_rot_3 = list([
            int(partial_graph.relative_rot_3[0] * self.codec_scale_factors["partial_graph"]["qx"]),
            int(partial_graph.relative_rot_3[1] * self.codec_scale_factors["partial_graph"]["qy"]),
            int(partial_graph.relative_rot_3[2] * self.codec_scale_factors["partial_graph"]["qz"]),
            int(partial_graph.relative_rot_3[3] * self.codec_scale_factors["partial_graph"]["qw"])
        ])
        encoded_msg.relative_rot_4 = list([
            int(partial_graph.relative_rot_4[0] * self.codec_scale_factors["partial_graph"]["qx"]),
            int(partial_graph.relative_rot_4[1] * self.codec_scale_factors["partial_graph"]["qy"]),
            int(partial_graph.relative_rot_4[2] * self.codec_scale_factors["partial_graph"]["qz"]),
            int(partial_graph.relative_rot_4[3] * self.codec_scale_factors["partial_graph"]["qw"])
        ])
        encoded_msg.relative_rot_5 = list([
            int(partial_graph.relative_rot_5[0] * self.codec_scale_factors["partial_graph"]["qx"]),
            int(partial_graph.relative_rot_5[1] * self.codec_scale_factors["partial_graph"]["qy"]),
            int(partial_graph.relative_rot_5[2] * self.codec_scale_factors["partial_graph"]["qz"]),
            int(partial_graph.relative_rot_5[3] * self.codec_scale_factors["partial_graph"]["qw"])
        ])
        # Verify they are all in the correct range
        if not (np.all(np.abs(encoded_msg.relative_rot_0) <= 127) and
                np.all(np.abs(encoded_msg.relative_rot_1) <= 127) and
                np.all(np.abs(encoded_msg.relative_rot_2) <= 127) and
                np.all(np.abs(encoded_msg.relative_rot_3) <= 127) and
                np.all(np.abs(encoded_msg.relative_rot_4) <= 127) and
                np.all(np.abs(encoded_msg.relative_rot_5) <= 127)):
            rospy.logerr("[%s] Partial graph relative rotations out of range!" % rospy.Time.now())
            return None
        # Encode the unique sigmas (covariance)
        # #TODO: esnure that the message is passed with sigmas6 and then we downselect to 3
        # unique_sigmas_0-5 is a uint8[6], multiply by scale, then convert to uint8
        encoded_msg.unique_sigmas_0 = list([
            int(partial_graph.unique_sigmas_0[0] * self.codec_scale_factors["partial_graph"]["sigma_x"]),
            int(partial_graph.unique_sigmas_0[1] * self.codec_scale_factors["partial_graph"]["sigma_y"]),
            int(partial_graph.unique_sigmas_0[2] * self.codec_scale_factors["partial_graph"]["sigma_yaw"])
        ])
        encoded_msg.unique_sigmas_1 = list([
            int(partial_graph.unique_sigmas_1[0] * self.codec_scale_factors["partial_graph"]["sigma_x"]),
            int(partial_graph.unique_sigmas_1[1] * self.codec_scale_factors["partial_graph"]["sigma_y"]),
            int(partial_graph.unique_sigmas_1[2] * self.codec_scale_factors["partial_graph"]["sigma_yaw"])
        ])
        encoded_msg.unique_sigmas_2 = list([
            int(partial_graph.unique_sigmas_2[0] * self.codec_scale_factors["partial_graph"]["sigma_x"]),
            int(partial_graph.unique_sigmas_2[1] * self.codec_scale_factors["partial_graph"]["sigma_y"]),
            int(partial_graph.unique_sigmas_2[2] * self.codec_scale_factors["partial_graph"]["sigma_yaw"])
        ])
        encoded_msg.unique_sigmas_3 = list([
            int(partial_graph.unique_sigmas_3[0] * self.codec_scale_factors["partial_graph"]["sigma_x"]),
            int(partial_graph.unique_sigmas_3[1] * self.codec_scale_factors["partial_graph"]["sigma_y"]),
            int(partial_graph.unique_sigmas_3[2] * self.codec_scale_factors["partial_graph"]["sigma_yaw"])
        ])
        encoded_msg.unique_sigmas_4 = list([
            int(partial_graph.unique_sigmas_4[0] * self.codec_scale_factors["partial_graph"]["sigma_x"]),
            int(partial_graph.unique_sigmas_4[1] * self.codec_scale_factors["partial_graph"]["sigma_y"]),
            int(partial_graph.unique_sigmas_4[2] * self.codec_scale_factors["partial_graph"]["sigma_yaw"])
        ])
        encoded_msg.unique_sigmas_5 = list([
            int(partial_graph.unique_sigmas_5[0] * self.codec_scale_factors["partial_graph"]["sigma_x"]),
            int(partial_graph.unique_sigmas_5[1] * self.codec_scale_factors["partial_graph"]["sigma_y"]),
            int(partial_graph.unique_sigmas_5[2] * self.codec_scale_factors["partial_graph"]["sigma_yaw"])
        ])
        # Verify they are all in the correct range
        if not (np.all(np.abs(encoded_msg.unique_sigmas_0) <= 255) and
                np.all(np.abs(encoded_msg.unique_sigmas_1) <= 255) and
                np.all(np.abs(encoded_msg.unique_sigmas_2) <= 255) and
                np.all(np.abs(encoded_msg.unique_sigmas_3) <= 255) and
                np.all(np.abs(encoded_msg.unique_sigmas_4) <= 255) and
                np.all(np.abs(encoded_msg.unique_sigmas_5) <= 255)):
            rospy.logerr("[%s] Partial graph unique sigmas out of range!" % rospy.Time.now())
            return None
        # Encode the measured ranges
        # meas_range 0-3 are uint16, multiply by scale, then convert to uint16
        encoded_msg.meas_range_0 = int(partial_graph.meas_range_0 * self.codec_scale_factors["partial_graph"]["range"])
        encoded_msg.meas_range_1 = int(partial_graph.meas_range_1 * self.codec_scale_factors["partial_graph"]["range"])
        encoded_msg.meas_range_2 = int(partial_graph.meas_range_2 * self.codec_scale_factors["partial_graph"]["range"])
        encoded_msg.meas_range_3 = int(partial_graph.meas_range_3 * self.codec_scale_factors["partial_graph"]["range"])
        # Verify they are all in the correct range
        if not (np.all(np.abs([encoded_msg.meas_range_0, encoded_msg.meas_range_1,
                             encoded_msg.meas_range_2, encoded_msg.meas_range_3]) <= 65535)):
            rospy.logerr("[%s] Partial graph measured ranges out of range!" % rospy.Time.now())
            return None
        # Return the encoded message
        return encoded_msg

    def decode_partial_graph_data_from_int(self, partial_graph: PartialGraph):
        """This function decodes the partial graph data from a message
        Args:
            partial_graph (PartialGraph): The partial graph message
        Returns:
            PartialGraph: The decoded partial graph message
        """
        # Decode the data by applying the reverse of the encoding process
        # Position is a int16[3], divide by scale to get original value
        partial_graph.relative_pos_0 = np.array([
            partial_graph.relative_pos_0[0] / self.codec_scale_factors["partial_graph"]["x_0"],
            partial_graph.relative_pos_0[1] / self.codec_scale_factors["partial_graph"]["y_0"],
            partial_graph.relative_pos_0[2] / self.codec_scale_factors["partial_graph"]["z_0"]
        ])
        # relative_pos 1-5 are int8[3], divide by scale to get original value
        for i in range(1, 6):
            partial_graph.__setattr__(f'relative_pos_{i}', np.array([
                getattr(partial_graph, f'relative_pos_{i}')[0] / self.codec_scale_factors["partial_graph"]["x"],
                getattr(partial_graph, f'relative_pos_{i}')[1] / self.codec_scale_factors["partial_graph"]["y"],
                getattr(partial_graph, f'relative_pos_{i}')[2] / self.codec_scale_factors["partial_graph"]["z"]
            ]))

        # Quaternion is a int8[4], divide by scale to get original value
        for i in range(0, 6):
            partial_graph.__setattr__(f'relative_rot_{i}', np.array([
                getattr(partial_graph, f'relative_rot_{i}')[0] / self.codec_scale_factors["partial_graph"]["qx"],
                getattr(partial_graph, f'relative_rot_{i}')[1] / self.codec_scale_factors["partial_graph"]["qy"],
                getattr(partial_graph, f'relative_rot_{i}')[2] / self.codec_scale_factors["partial_graph"]["qz"],
                getattr(partial_graph, f'relative_rot_{i}')[3] / self.codec_scale_factors["partial_graph"]["qw"]
            ]))

        # Sigmas is a uint8[6], divide by scale to get original value
        for i in range(0, 6):
            partial_graph.__setattr__(f'unique_sigmas_{i}', np.array([
                getattr(partial_graph, f'unique_sigmas_{i}')[0] / self.codec_scale_factors["partial_graph"]["sigma_x"],
                getattr(partial_graph, f'unique_sigmas_{i}')[1] / self.codec_scale_factors["partial_graph"]["sigma_y"],
                getattr(partial_graph, f'unique_sigmas_{i}')[2] / self.codec_scale_factors["partial_graph"]["sigma_yaw"]
            ]))
        # Measured ranges are uint16, divide by scale to get original value
        partial_graph.meas_range_0 = int(partial_graph.meas_range_0 / self.codec_scale_factors["partial_graph"]["range"])
        partial_graph.meas_range_1 = int(partial_graph.meas_range_1 / self.codec_scale_factors["partial_graph"]["range"])
        partial_graph.meas_range_2 = int(partial_graph.meas_range_2 / self.codec_scale_factors["partial_graph"]["range"])
        partial_graph.meas_range_3 = int(partial_graph.meas_range_3 / self.codec_scale_factors["partial_graph"]["range"])
        # Return the decoded message
        return partial_graph

    # Message Handling Functions
    def send_test_init_prior(self, target_addr):
        """This function sends an initial prior factor to the estimator
        Args:
            target_addr (int): the target address
        """
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1])
        # Generate the initial prior factor message
        initial_position, initial_orientation, initial_sigmas = self.generate_test_init_prior_data("zero")
        if initial_position is None or initial_orientation is None or initial_sigmas is None:
            rospy.logerr("[%s] Failed to generate initial prior factor data!" % rospy.Time.now())
            return
        # Encode the initial prior factor data into a message
        initial_position, initial_orientation, initial_sigmas = self.encode_init_prior_data_as_int(initial_position, initial_orientation, initial_sigmas)
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
        # Convert the target_address to a chr
        target_chr = chr(ord("A") + target_addr)
        # Publish the message to the target address
        self.init_prior_pubs[target_chr].publish(init_prior_msg)
        # Log that we've sent the message
        rospy.loginfo("[%s] Published Initial Prior Factor to %s" % (rospy.Time.now(), target_addr))
        return

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
        # Log the initial prior factor
        rospy.loginfo("[%s] Received Initial Prior Factor from %s" % (rospy.Time.now(),local_addr))
        return

    def send_test_partial_graph(self, target_addr):
        """This function sends a partial graph to the estimator
        Args:
            target_addr (int): the target address
        """
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1])
        # Build the partial graph message
        partial_graph_msg = self.generate_test_partial_graph_data(method="zero")
        if partial_graph_msg is None:
            rospy.logerr("[%s] Failed to generate test partial graph!" % rospy.Time.now())
            return
        # Encode the partial graph data
        partial_graph_msg = self.encode_partial_graph_data_as_int(partial_graph_msg)
        if partial_graph_msg is None:
            rospy.logerr("[%s] Failed to encode partial graph data!" % rospy.Time.now())
            return
        # Convert the target_address to a chr
        target_chr = chr(ord("A") + target_addr)
        # Publish the message to the target address
        self.partial_graph_pubs[target_chr].publish(partial_graph_msg)
        # Log that we've sent the message
        rospy.loginfo("[%s] Published Partial Graph to %s" % (rospy.Time.now(), target_addr))
        return

    def on_partial_graph(self, msg):
        """This function processes partial graph messages
        Args:
            msg (PartialGraph): The partial graph message
        """
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1])
        # Get the key for the remote agent
        remote_symbol = msg.local_addr
        # Log the partial graph
        rospy.loginfo("[%s] Received Partial Graph from %s" % (rospy.Time.now(), remote_symbol))
        return

    def run_cycle(self):
        """This function runs the comms cycle
        """
        # Get the target agent and landmark for the cycle
        # target_agent, targe[t_landmark = self.get_cycle_targets()
        cycle_targets = self.cycle_target_mapping[self.tdma_status.current_slot]
        target_agent = cycle_targets[0]
        target_landmark = cycle_targets[1]
        # Log the current cycle status
        #rospy.loginfo("[%s] Running Cycle with Target Agent %s and Landmark %s" % (rospy.Time.now(), target_agent, target_landmark))
        if self.cycle_status:
            if target_agent != []:
                rospy.loginfo("[%s] Sending Partial Graph to Agent %s" % (rospy.Time.now(), target_agent))
                if self.tdma_cycle_sequence == 0:
                    #self.send_test_init_prior(target_agent)
                    self.send_test_partial_graph(target_agent)
                else:
                    #self.send_test_init_prior(target_agent)
                    self.send_test_partial_graph(target_agent)
            #     rospy.loginfo("[%s] Sending Ping to Agent %s" % (rospy.Time.now(), target_agent))
            #     self.send_ping(target_agent)
            # elif target_landmark != []:
            #     rospy.loginfo("[%s] Sending Ping to Landmark %s" % (rospy.Time.now(), target_landmark))
            #     self.send_ping(target_landmark)
            else:
                pass
            # Log that we've completed the cycle
            self.cycle_status = False
            self.tdma_cycle_sequence += 1
            rospy.loginfo("[%s] Cycle Complete, %s sec remaining in slot" % (rospy.Time.now(), self.tdma_status.remaining_slot_seconds))
        else:
            pass
        return

    def run(self):
        """This function runs the comms cycle test
        """
        # Wait for the node to be ready
        rospy.loginfo("[%s] Starting Comms Cycle Test" % rospy.Time.now())
        self.on_startup()
        rospy.loginfo("[%s] Comms Cycle Test Ready" % rospy.Time.now())
        # Run the cycle until the node is shut down
        while not rospy.is_shutdown():
            self.run_cycle()
            rospy.sleep(0.1)
        rospy.loginfo("[%s] Comms Cycle Test Stopped" % rospy.Time.now())
        return

if __name__ == "__main__":
    try:
        cycle_test = CycleTest()
        cycle_test.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[%s] Comms Cycle Test Interrupted" % rospy.Time.now())
    except Exception as e:
        rospy.logerr("[%s] Comms Cycle Test Error: %s" % (rospy.Time.now(), e))
    finally:
        rospy.loginfo("[%s] Comms Cycle Test Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Comms Cycle Test Exiting")
        exit(0)
# End of File