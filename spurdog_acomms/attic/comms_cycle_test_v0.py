#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from ros_acomms_msgs.msg import(
    TdmaStatus, TdmaAdvancedStatus, SoundSpeedProfile
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingTranspondersRequest,
)
from spurdog_acomms.msg import(
    PosePriorFactor, PartialGraph, RelativePoseMin, InitiatedRange, RecievedRange
)
from std_msgs.msg import String, Time

class CycleTest:
    """ This is a node to test the comms capability dockside
    It will continuously iterate the comms cycle between an agent and some landmarks
    It won't actually pass data, just template packets.
    """
    def __init__(self):
        rospy.init_node('comms_cycle_test', anonymous=True)
        self.local_address = rospy.get_param("modem_address", 0)
        self.num_agents = rospy.get_param("num_agents", 2)
        self.num_landmarks = rospy.get_param("num_landmarks", 0)
        self.landmarks = rospy.get_param("landmarks", {}) # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = rospy.get_param("sound_speed", 1500)
        self.sigma_range = rospy.get_param("sigma_range", 1)
        self.sigma_depth = rospy.get_param("sigma_depth", 1)
        self.sigma_roll = rospy.get_param("sigma_roll", 0.1)
        self.sigma_pitch = rospy.get_param("sigma_pitch", 0.1)
        self.tdma_status = TdmaStatus()
        self.modem_addresses = {}
        self.ping_sent_time = None
        self.acomms_event_time = None
        self.cycle_status = 0
        self.tdma_cycle_sequence = 0
        self.cycle_tgt_agent = None
        self.cycle_tgt_ldmk = None

        self.init_prior_pubs = {}
        self.init_prior_subs = {}
        self.partial_graph_pubs = {}
        self.partial_graph_subs = {}
        
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
        #self.gps_pose = rospy.Subscriber("garmin18x_node/fix", NavSatFix, self.on_gps_fix)
        self.message_test_pub = rospy.Publisher("test_ssp", SoundSpeedProfile, queue_size=10)
        self.message_test_sub = rospy.Subscriber("from_acomms/test_ssp", SoundSpeedProfile, self.on_test_message)

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
            for i in range(self.num_landmarks):
                address = i + self.num_landmarks
                self.modem_addresses["L%d" % i] = [address,0]
        rospy.loginfo("[%s] Modem Addresses: %s" % (rospy.Time.now(), self.modem_addresses))
        return

    def on_startup(self):
        """This function initializes the modem addresses and sets up the initial relative pose measurements
        """
        self.setup_addresses()
        # Initialize the subs/pubs for the test messages
        # for key in self.modem_addresses:
        #     if key[0] != "L" or key[1] != self.local_address:
        #         #self.init_prior_pubs[key] = rospy.Publisher("modem/init_prior_%s" % key, PosePriorFactor, queue_size=10)
        #         #self.partial_graph_pubs[key] = rospy.Publisher("modem/partial_graph_%s" % key, PartialGraph, queue_size=10)
        # for key in self.modem_addresses:
        #     if key[0] != "L" and key[1] != self.local_address:
        #         #self.init_prior_subs[key] = rospy.Subscriber("modem/from_acomms/init_prior_%s" % key, PosePriorFactor, self.on_init_prior)
        #         #self.partial_graph_subs[key] = rospy.Subscriber("modem/from_acomms/partial_graph_%s" % key, PartialGraph, self.on_partial_graph)
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
                rospy.loginfo("[%s] TDMA Slot Started, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
                self.cycle_status = 0
                self.send_ping(1)
            elif we_are_active == True and (round(remaining_sec_in_slot,0) == 10):
                rospy.loginfo("[%s] TDMA Slot Active, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
                self.build_test_message()
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
        if nmea_type == "$CAMPC" or nmea_type == "$CAMUC":
            self.ping_sent_time = rospy.Time.now()
            self.cycle_status +=1
        elif nmea_type == "$CAMPA" or nmea_type == "$CAMUA":
            self.acomms_event_time = rospy.Time.now()
            self.acomms_event.publish(self.acomms_event_time)
            self.modem_addresses[chr(ord("A") + self.local_address)][1] += 1
            self.on_ping_ack(nmea_type, data)
        elif nmea_type == "$CAMPR" or nmea_type == "$CAMUR":
            self.acomms_event_time = (rospy.Time.now() - self.ping_sent_time) / 2
            self.acomms_event.publish(self.acomms_event_time)
            self.modem_addresses[chr(ord("A") + self.local_address)][1] += 1
            rospy.loginfo("[%s] Received Ping Response from %s" % (rospy.Time.now(), data[0]))
            self.cycle_status += 1
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
        # ping_req = PingModemRequest()
        # ping_req.dest = target_addr
        # ping_req.rate = 1
        # ping_req.cdr = 0
        # ping_req.hexdata = [0,1,0,0]  # Placeholder for the payload
        # #ping_req.hexdata = self.pack_symbol_to_ping_payload(symbol)
        # ping_req.timeout_sec = 5

        # try:
            # send the ping and try to get a response
        #rospy.loginfo("[%s] One Ping Only Vasily." % (rospy.Time.now()))
            #ping_resp = self.ping_client(ping_req)
        ping_nmea = "$CCMPC,%s,%s" % (self.local_address, target_addr)
        self.nmea_to_modem.publish(ping_nmea)
        # Wait 5sec
        rospy.sleep(5)
        self.build_test_message()
            # if ping_resp.timed_out:
            #     rospy.logwarn("[%s] Ping to %s Timed Out" % (rospy.Time.now(), target_addr))
            # else:
            #     rospy.loginfo("[%s] Ping to %s Successful" % (rospy.Time.now(), target_addr))
            #     src = ping_resp.cst.src
            #     dest = ping_resp.cst.dest
            #     owtt = ping_resp.one_way_travel_time
            #     measured_range = owtt * self.sound_speed
            #     timestamp = ping_resp.timestamp
            #     # Get the key association from the src and dest address
            #     src_symbol = chr(ord("A") + src)
            #     dest_chr = chr(ord("A") + dest)
            #     src_key = src_symbol + str(self.modem_addresses[src_symbol][1]+1)

        # except rospy.ServiceException as e:
        #     rospy.logerr("[%s] Ping Service Call Failed: %s" % (rospy.Time.now(), e))
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
            #hex_payload = data[2].split("*")[0]
        else:
            dest = data[1].split("*")[0]
        # Convert the hex payload to a symbol
        remote_symbol = 1
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1]+1)
        time = rospy.Time.now()
        # Log the ping ack
        rospy.loginfo("[%s] Received Ping Ack from %s to %s, with symbol %s" % (ack_time, src, dest, remote_symbol))

        return
        # Packing/Unpacking Functions:
    
    # Message Handling:
    def send_init_prior(self, target_addr):
        """This function sends an initial prior factor to the estimator
        Args:
            target_addr (int): the target address
        """
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1])
        # Set the initial prior factor message
        init_prior_msg = PosePriorFactor()
        init_prior_msg.key = local_symbol
        init_prior_msg.pose.translation = [20, 20, 2]
        init_prior_msg.pose.orientation = [0, 0, 0, 1]
        init_prior_msg.noise = [1, 1, 1, 1, 1, 1]
        # Publish the message
        self.init_prior_pubs[target_addr].publish(init_prior_msg)
        # Log that we've sent the message
        rospy.loginfo("[%s] Published Initial Prior Factor to %s" % (rospy.Time.now(), target_addr))
        return
    
    def send_partial_graph(self, target_addr):
        """This function sends a partial graph to the estimator
        Args:
            target_addr (int): the target address
        """
        # Make the relative pose message
        test_relative_pose = RelativePoseMin()
        test_relative_pose.x = 1
        test_relative_pose.y = 1
        test_relative_pose.z = 1
        test_relative_pose.qx = 1
        test_relative_pose.qy = 1
        test_relative_pose.qz = 1
        test_relative_pose.qw = 1
        test_relative_pose.sigma_x = 10
        test_relative_pose.sigma_y = 10
        test_relative_pose.sigma_yaw = 1
        # Make the initiated range message
        test_initiated_range = InitiatedRange()
        test_initiated_range.local_index = 0
        test_initiated_range.remote_index = 1
        test_initiated_range.measured_range = 10
        # Make the received range message
        test_received_range = RecievedRange()
        test_received_range.local_index = 0
        test_received_range.remote_addr = 1
        test_received_range.remote_index = 11
        # Set the partial graph message
        partial_graph_msg = PartialGraph()
        partial_graph_msg.local_addr = self.local_address
        partial_graph_msg.full_index = 11
        partial_graph_msg.num_poses = 6
        partial_graph_msg.rel_pose_0 = test_relative_pose
        partial_graph_msg.rel_pose_1 = test_relative_pose
        partial_graph_msg.rel_pose_2 = test_relative_pose
        partial_graph_msg.rel_pose_3 = test_relative_pose
        partial_graph_msg.rel_pose_4 = test_relative_pose
        partial_graph_msg.rel_pose_5 = test_relative_pose
        partial_graph_msg.init_range_0 = test_initiated_range
        partial_graph_msg.init_range_1 = test_initiated_range
        partial_graph_msg.init_range_2 = test_initiated_range
        partial_graph_msg.init_range_3 = test_initiated_range
        partial_graph_msg.rcvd_range_0 = test_received_range
        partial_graph_msg.rcvd_range_1 = test_received_range
        partial_graph_msg.sigma_range = 3
        # Publish the message
        self.partial_graph_pubs[target_addr].publish(partial_graph_msg)
        # Log that we've sent the message
        rospy.loginfo("[%s] Published Partial Graph to %s" % (rospy.Time.now(), target_addr))
        return
    
    def on_init_prior(self, msg):
        """This function processes initial prior factor messages
        Args:
            msg (PosePriorFactor): The initial prior factor message
        """
        # Get the key for the local agent
        local_chr = chr(ord("A") + self.local_address)
        local_symbol = local_chr + str(self.modem_addresses[local_chr][1])
        # Get the key for the remote agent
        remote_symbol = msg.key
        # Log the initial prior factor
        rospy.loginfo("[%s] Received Initial Prior Factor from %s" % (rospy.Time.now(), remote_symbol))
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
    
    def build_test_message(self):
    #         - id: 103
    #   message_codec: default
    #   subscribe_topic: "command"
    #   publish_topic: "from_acomms/command"
    #   ros_type: "ros_acomms_msgs/PingTranspondersRequest"
    #   default_dest: 221
    #   queue_order: fifo
    #   queue_maxsize: 1
    #   priority: 100
    #   fields:
    #     transponder_dest_mask:
    #       codec: fixed_len_array
    #       element_type: bool
    #       length: 4
    #     timeout_sec:
    #       codec: float
    #       min_value: -1.0
    #       max_value: 100.0
    #       precision: 1
    # Build a template message that uses this codec
        """This function builds a template command message for the ping transponders
        """
        # Create the message
        test_msg = SoundSpeedProfile()
        test_msg.depths = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]  # Example depths
        test_msg.sound_speeds = [
            1500, 1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508, 1509, 1510
        ]
        test_msg.water_depth = 10.0  # Example water depth
        self.message_test_pub.publish(test_msg)
        # Publish the message
        rospy.loginfo("[%s] Published Test Message" % (rospy.Time.now()))
        return

    def on_test_message(self, msg):
        """This function processes the test message from the modem
        Args:
            msg (PingTranspondersRequest): The test message
        """
        # Get the transponder destination mask and timeouts
        depths = msg.depths
        sound_speeds = msg.sound_speeds
        water_depth = msg.water_depth
        # Log the test message
        rospy.loginfo("[%s] Received Test Message: Depths: %s, Sound Speeds: %s, Water Depth: %s" % (rospy.Time.now(), depths, sound_speeds, water_depth))
        # Process the test message
        # For now, just log the message
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
        # if self.tdma_status.we_are_active:
        #     if self.cycle_status == 0: # At cycle start
        #         # Set the target agent and target landmark
        #         target_agent, target_landmark = self.get_cycle_targets()
        #         # log the target agent and landmark
        #         rospy.loginfo("[%s] Target Agent: %s, Target Landmark: %s" % (rospy.Time.now(), target_agent, target_landmark))
        #         # Send a ping to the target agent
        #         self.send_ping(target_agent)
        #     elif self.cycle_status == 1:
        #         # Check for ping timeout
        #         if (rospy.Time.now() - self.ping_sent_time).to_sec() > 5:
        #             # Abort the ping, move on to the next step
        #             rospy.logwarn("[%s] Ping to %s Timed Out" % (rospy.Time.now(), target_agent))
        #             self.cycle_status += 1
        #         else:
        #             pass
        #     elif self.cycle_status == 2:
        #         # Send the partial graph to the target agent
        #         if self.tdma_cycle_sequence % 2 == 0:
        #             # Send the initial priors
        #             #self.send_init_prior(target_agent)
        #             self.build_test_message()
        #             self.cycle_status += 1
        #         else:
        #             # Send the partial graph
        #             #self.send_partial_graph(target_agent)
        #             self.build_test_message()
        #             self.cycle_status += 1
        #     else:
        #         # We are at the end of the cycle do nothing
        #         pass
        # else:
        #     # We are not active, do nothing
        #     pass   
        rospy.spin()
        return
    
def main():
    manager = CycleTest()
    manager.on_startup()
    while not rospy.is_shutdown():
        manager.run()

if __name__ == '__main__':
    main()