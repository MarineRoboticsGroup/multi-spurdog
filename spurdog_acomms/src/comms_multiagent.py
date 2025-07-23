#!/usr/bin/env python3
import rospy
import csv
from os.path import dirname, join, abspath
import numpy as np
# from datetime import datetime
# import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32, Bool
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from ros_acomms_msgs.msg import(
    TdmaStatus, QueueStatus, PingReply, CST, XST, ReceivedPacket
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)
from spurdog_acomms.msg import(
    Bar30SoundSpeed, RangeFactorStamped, PoseFactorStamped,
    AcommsCycleStatus, ReceivedSignalStats,
    BasicGraphUpdate, AdvancedGraphUpdate,
)
from spurdog_acomms.srv import(
    PreintegrateImu, PreintegrateImuResponse
)
from spurdog_acomms_utils.setup_utils import(
    configure_modem_addresses,
    configure_cycle_targets
)
from spurdog_acomms_utils.nmea_utils import (
    parse_nmea_sentence,
    parse_nmea_cacmd,
    parse_nmea_cacma,
    parse_nmea_cacmr,
    parse_nmea_carfp,
    parse_nmea_carev,
    parse_nmea_catxf
)
from spurdog_acomms_utils.codec_utils import (
    encode_pwcs_as_int,
    encode_range_event_as_int,
    decode_pwc_from_int,
    decode_range_event_from_int
)

class CycleManager:
    """This is a node to run the comms cycle for the vehicle.
        - This is a lightweight version designed to obtain the maximum amount of range data
        - It is designed to run on multiple agent with an arbitary number of landmarks
        - Agents will observe tdma, and ping each other (and landmarks) in round-robin fashions
        - This is configured to ping one target, get a response and immediately ping the next target
    """
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        # Get current namespace, e.g. "/actor_0/comms_lbl"
        full_ns = rospy.get_namespace()           # e.g., "/actor_0/comms_lbl/"
        rospy.loginfo("[%s] Full Namespace as %s"%(rospy.Time.now(), full_ns))
        # Config:
        self.local_address = int(rospy.get_param(full_ns + "modem_address", 0))
        self.num_agents = int(rospy.get_param(full_ns + "num_agents", 2))
        self.num_landmarks = int(rospy.get_param(full_ns + "num_landmarks", 2))
        self.landmarks = {
            "L0": rospy.get_param(full_ns + "landmarks/L0"),
            "L1": rospy.get_param(full_ns + "landmarks/L1")
        }
        self.modem_addresses = {}
        self.address_to_name = {}
        self.cycle_target_mapping = {}
        self.planned_targets = []
        # Subscribed variables
        self.bar30_sound_speed = 0
        self.in_water = False
        self.depth = 0.0
        self.gps_fix = [[1,2,3],[0,0,0,1],[1.7,1.7,3.5,0.1,0.1,0.1]] # [position, orientation, covariance]
        # Variables for acomms:
        self.ping_method = "ping with payload"
        self.cycle_mode = rospy.get_param("~cycle_mode", "both")  # Can be "normal" or "aggressive"
        self.send_data = rospy.get_param("~send_data", False)  # Whether to send data or not
        self.message_mode = rospy.get_param("~message_mode", "basic")  # Can be "basic" or "advanced"
        self.ping_timeout = float(rospy.get_param("~ping_timeout", 4))
        self.sound_speed = float(rospy.get_param("~sound_speed", 1500))
        self.range_sigma = float(rospy.get_param("~sigma_range", 1)) # meters, this is the expected error in the range measurement
        self.tdma_status = TdmaStatus()
        self.active_slot_seq = 0
        # Variables for Logging:
        self.pose_time_lookup = {}
        self.cycle_status = {
            "pings_attempted": 0,
            "pings_successful": 0,
            "min_range": 1000.0,
            "max_range": -1000.0,
            "last_range_timestamp": rospy.Time.now(),
            "last_range_interval": rospy.Duration(0),
            "last_range_target": 0,
            "last_range_distance": 0.0
        }
        self.partial_ranges = []
        # partial ranges is a list of dicts, each with the following keys:
        # timestamp, key1: str, key2: str, remote_address: int, index: int, measured_range: float, sigma_range: float, depth1: float, depth2: float
        # Check services
        rospy.loginfo("[%s] Waiting for services..." % rospy.Time.now())
        rospy.wait_for_service("modem/ping_modem")
        self.ping_client = rospy.ServiceProxy("modem/ping_modem", PingModem)
        rospy.wait_for_service("preintegrate_imu")
        self.preintegrate_imu = rospy.ServiceProxy("preintegrate_imu", PreintegrateImu)
        rospy.loginfo("[%s] Services ready, initializing topics" % rospy.Time.now())

        # Initialize topics
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        self.range_logging_sub = rospy.Subscriber("modem/ping_reply",PingReply, self.on_range_log)
        self.xst = rospy.Subscriber("modem/xst", XST, self.on_xst)
        self.gps = rospy.Subscriber("gps", PoseWithCovarianceStamped, self.on_gps)
        self.in_water_sub = rospy.Subscriber("in_water", Bool, self.on_in_water)
        self.sound_speed_sub = rospy.Subscriber("bar30/sound_speed", Bar30SoundSpeed, self.on_sound_speed)
        self.nav_state_sub = rospy.Subscriber("nav_state",PoseStamped, self.on_nav_state)
        self.tdma_status_sub = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_from_modem)

        self.queue_status_sub = rospy.Subscriber("modem/queue_status", QueueStatus, self.on_queue_status)
        self.packet_rcv_sub = rospy.Subscriber("modem/packet_rx", ReceivedPacket, self.on_nmea_from_modem)
        if self.message_mode == "basic":
            self.graph_update_msg = BasicGraphUpdate()
            self.graph_update_pub = rospy.Publisher("modem/to_acomms/basic_graph_update", BasicGraphUpdate, queue_size=1)
            self.graph_update_sub = rospy.Subscriber("modem/from_acomms/basic_graph_update", BasicGraphUpdate, self.on_graph_update)
        elif self.message_mode == "advanced":
            self.graph_update_msg = AdvancedGraphUpdate()
            self.graph_update_pub = rospy.Publisher("modem/to_acomms/adv_graph_update",AdvancedGraphUpdate, queue_size=1)
            self.graph_update_sub = rospy.Subscriber("modem/from_acomms/adv_graph_update", AdvancedGraphUpdate, self.on_graph_update)
        self.acomms_event_pub = rospy.Publisher("led_command", String, queue_size=1)
        self.range_factor_pub = rospy.Publisher("range_factor", RangeFactorStamped, queue_size=1)
        self.pose_factor_pub = rospy.Publisher("pose_factor", PoseFactorStamped, queue_size=1)
        self.cycle_status_pub = rospy.Publisher("comms_cycle_status", AcommsCycleStatus, queue_size=1)
        self.recieved_signal_stats_pub = rospy.Publisher("received_signal_stats", ReceivedSignalStats, queue_size=1)

        # Initialize the modem addresses and cycle targets
        rospy.loginfo("[%s] Topics ready, initializing comms cycle" % rospy.Time.now())
        self.configure_comms_cycle()
        rospy.loginfo("[%s] Comms Cycle Configured" % rospy.Time.now())
        rospy.sleep(10) # allow for modem to configure
        rospy.loginfo("[%s] Starting Comms Cycle" % rospy.Time.now())
        rospy.Timer(rospy.Duration(1.0), self.send_acomms_status)
        # self.send_ping(self.cycle_target_mapping[0][0])  # Start the ping cycle with the first target

    def configure_comms_cycle(self):
        """This function configures the comms cycle for the vehicle.
        """
        # Get the modem addresses and cycle targets
        self.modem_addresses = configure_modem_addresses(self.num_agents, self.num_landmarks, self.local_address)
        self.address_to_name = {v[0]: k for k, v in self.modem_addresses.items()}
        rospy.loginfo("[%s] Address to name: %s" % (rospy.Time.now(), self.address_to_name))
        # make a list of modem addresses that don't include the local address (the int address, not the name)
        if self.cycle_mode == "both":
            ping_addresses = [value[0] for value in self.modem_addresses.values() if value[0] != self.local_address]
        elif self.cycle_mode == "agents":
            # Only ping the agents, not the landmarks
            ping_addresses = [value[0] for value in self.modem_addresses.values() if value[0] < self.num_agents-1 and value[0] != self.local_address]
        elif self.cycle_mode == "landmarks":
            # Only ping the landmarks, not the agents
            ping_addresses = [value[0] for value in self.modem_addresses.values() if value[0] >= self.num_agents and value[0] != self.local_address]
        else:
            rospy.logerr("[%s] Invalid cycle mode: %s" % (rospy.Time.now(), self.cycle_mode))
            return
        # Configure a list of cycle targets to ping (the modem addresses that don't include the local address)
        self.cycle_target_mapping = {
            "0": ping_addresses,
            "1": ping_addresses[::-1]
        }
        local_chr = chr(ord("A") + self.local_address)
        self.pose_time_lookup[local_chr + str(0)] = rospy.Time.now()  # Initial pose at time of cycle start
        rospy.loginfo("[%s] Cycle Targets: %s" % (rospy.Time.now(), self.cycle_target_mapping))
        return

    # Ping Handling:
    def on_nmea_from_modem(self, msg):
        """This function receives NMEA messages from the modem
        """
        if isinstance(msg, ReceivedPacket):
            rospy.loginfo("[%s] Recieved Packet from %s."%(rospy.Time.now(), msg.packet.src))
            return
        nmea_type, data = parse_nmea_sentence(msg.data)
        # Rekey the modem addesses to be {address: [Name, index]}
        # Process the NMEA data by field
        if nmea_type == "$CACMD": # Modem-to-host acknowledgement of a ping command
            src, dest = parse_nmea_cacmd(data)
            if data[0] == "PNG" and src == self.local_address:
                self.acomms_event_pub.publish("priority=2,pattern=([0.0.0.255]:0.5)([0.0.255.0]:1.0),cycles=1")
                rospy.loginfo("[%s] Sent Ping to %s" % (rospy.Time.now(), self.address_to_name[dest]))
            else:
                rospy.logerr("[%s] Received $CACMD with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMA": # Modem-to-host acknowledgement of a ping recieved
            src, dest, recieved_ping_time = parse_nmea_cacma(data)
            # Convert time (ROS Time in sec) to ROS Time in nsec
            rcvd_stamp = rospy.Time.from_sec(recieved_ping_time)
            if data[0] == "PNG" and dest == self.local_address:
                # Log to the partial ranges:
                self.request_preintegration(rcvd_stamp, True) # Request a relative pose measurement
                self.acomms_event_pub.publish("priority=2,pattern=([0.0.0.255]:0.5)([100.0.150.50]:1.0),cycles=1")
                rospy.loginfo("[%s] Received Ping from %s" % (recieved_ping_time, self.address_to_name[src]))
            elif data[0] == "PNG":
                rospy.loginfo("[%s] Overheard Ping from %s to %s" % (recieved_ping_time, self.address_to_name[src], self.address_to_name[dest]))
            else:
                rospy.loginfo("[%s] Received $CACMA with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMR": # Modem-to-host acknowledgement of a ping response
            src, dest, recieved_ping_time, owtt = parse_nmea_cacmr(data)
            if data[0] == "PNR" and src == self.local_address:
                #self.acomms_event_pub.publish("priority=2,pattern=([0.0.0.255]:0.5)([0.255.0.50]:1.0),cycles=1")
                measured_range = owtt * self.sound_speed
                self.add_range_event_to_graph_update(dest, None, measured_range, sigma_range=self.range_sigma)
                rospy.loginfo("[%s] Received Ping Response from %s" % (recieved_ping_time, self.address_to_name[dest]))
            elif data[0] == "PNR":
                rospy.loginfo("[%s] Overheard Ping Response from %s to %s" % (recieved_ping_time, self.address_to_name[src], self.address_to_name[dest]))
            else:
                rospy.logerr("[%s] Received $CACMR with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CARFP": # Modem-to-host acknowledgement of a minipacket ping payload
            src, dest, recieved_msg_time, num_frames, payload = parse_nmea_carfp(data)
            #rospy.loginfo("[%s] CARFP data payload: %s" % (rospy.Time.now(), payload))
            if recieved_msg_time == None or src == None or dest == None:
                #rospy.logerr("[%s] CARFP message is missing required fields" % rospy.Time.now())
                return
            elif dest == self.local_address:
                    if payload == '':
                        pass
                    else:
                        rospy.loginfo("[%s] Received Ping from %s with payload %s" % (recieved_msg_time, self.address_to_name[src], payload))
                        # Analyze the payload (should be a letter followed by a integer, e.g. "A1")
                        self.partial_ranges.append({
                            "timestamp": recieved_msg_time,
                            "key1": chr(ord("A") + self.local_address) + str(self.modem_addresses[chr(ord("A") + self.local_address)][1]),
                            "key2": payload,  # e.g. "L0" or "A2"
                            "remote_address": src,
                            "index": None,  # This will be filled later when the msg is received
                            "measured_range": None,  # This will be filled later when the msg is received
                            "sigma_range": None,    # This will be filled later when the msg is received
                            "depth1": self.depth,
                            "depth2": None  # This will be filled later when the msg is received
                        })
                        self.add_range_event_to_graph_update(src, int(payload[1:]), None, None)

            elif src == dest:
                rospy.loginfo("[%s] Received packet $CARFP from %s" % (recieved_msg_time, self.address_to_name[src]))
            elif dest != self.local_address:
                rospy.loginfo("[%s] Overheard Ping-related $CARFP from %s to %s with paylaod %s" % (recieved_msg_time, self.address_to_name[src], self.address_to_name[dest], payload))
            else:
                rospy.logerr("[%s] Received $CARFP with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CAREV" and self.ping_method == None: # Modem-to-host $CAREV message to determine the firmware version
            firmware_version = parse_nmea_carev(data)
            # if firmware_version[0] == "3":
            #     # New deckbox firmware
            #     self.ping_method = "ping with payload"
            # else:
            #     # Old deckbox firmware
            #     self.ping_method = "no payload"
        elif nmea_type == "$CATXP": # Modem-to-host report of beginning transmission
            # This is a transmit report, we can ignore it for now
            #rospy.loginfo("[%s] Received $CATXP message: %s" % (rospy.Time.now(), data))
            pass
        elif nmea_type == "$CATXF": # Modem-to-host report of end of transmission
            nbytes = parse_nmea_catxf(data)
            if nbytes > 2:
                #self.acomms_event_pub.publish("priority=2,pattern=([255.255.0.0]:1.0),cycles=3")
                pass
        else:
            return
        return

    def on_tdma_from_modem(self, msg: TdmaStatus):
        """This function receives the TDMA status from the modem
        Args:
            msg (TdmaStatus): The TDMA status message
        """
        # Get fields from the TDMA status message
        status_time = msg.header.stamp
        current_slot = msg.current_slot
        we_are_active = msg.we_are_active
        remaining_slot_seconds = msg.remaining_slot_seconds
        remaining_active_seconds = msg.remaining_active_seconds
        time_to_next_active = msg.time_to_next_active
        time_to_next_dedicated_slot = msg.time_to_next_dedicated_slot
        slot_duration_seconds = msg.slot_duration_seconds
        # Check if we are active and were not previously
        if we_are_active ==True and self.tdma_status.we_are_active == False:
            rospy.loginfo("[%s] We are now active in the TDMA cycle" % status_time)
            # Send a ping to the first target in the cycle
            self.planned_targets = list(self.cycle_target_mapping["0"].copy())
            rospy.loginfo("Planned Targets: %s" % self.planned_targets)
            self.tdma_status = msg #NOTE: This must be here for send_ping to execute properly
            self.send_ping(self.planned_targets[0])
        else:
            self.tdma_status = msg
        return

    def on_queue_status(self, msg: QueueStatus):
        """This function receives the queue status from the modem
        Args:
            msg (QueueStatus): The queue status message
        """
        timestamp = msg.header.stamp
        msg_in_queue = msg.total_message_count
        if msg_in_queue > 1:
            rospy.logwarn("[%s] %d messages in the modem queue" % (timestamp, msg_in_queue))
        else:
            pass
        return

    def on_graph_update(self, msg):
        """This function receives test data from the modem
        Args:
            msg (BasicGraphUpdate or AdvancedGraphUpdate): The graph update message
        """
        rospy.logwarn("[%s] Processing Graph Update from %s" % (rospy.Time.now(), self.address_to_name[msg.sender_address]))
        self.acomms_event_pub.publish("priority=2,pattern=([0.0.0.255]:0.5)([100.0.150.50]:2.0),cycles=1")
        # Parse the graph update into PoseFactorStamped and RangeFactorStamped messages
        sender_address = msg.sender_address
        initial_key_index = msg.first_key_index
        num_poses = msg.num_poses
        # Build a list of pose keys to publish:
        pose_keys = []
        for i in range(num_poses+1): #0,1,2,3,4
            key_index = initial_key_index + i
            pose_key = chr(ord("A") + sender_address) + str(key_index)
            pose_keys.append(pose_key)
        # Now, iterate over the poses and publish them
        for i in range(num_poses): #0,1,2,3
            key1 = pose_keys[i]
            key2 = pose_keys[i+1]
            if self.message_mode == "advanced":
                prefix = f"pose_prior_0_"
                x = getattr(msg, prefix + "x")
                y = getattr(msg, prefix + "y")
                z = getattr(msg, prefix + "z")
                qw = getattr(msg, prefix + "qw")
                qx = getattr(msg, prefix + "qx")
                qy = getattr(msg, prefix + "qy")
                qz = getattr(msg, prefix + "qz")
                sigma_x = getattr(msg, prefix + "sigma_x")
                sigma_y = getattr(msg, prefix + "sigma_y")
                sigma_z = getattr(msg, prefix + "sigma_z")
                sigma_psi = getattr(msg, prefix + "sigma_psi")
                encoded_pose = [x, y, z, qw, qx, qy, qz, sigma_x, sigma_y, sigma_z, sigma_psi, 0, 0, 0]
                prior = decode_pwc_from_int(encoded_pose, "prior")
                # Create a PoseFactorStamped message
                pose_factor_msg = PoseFactorStamped()
                pose_factor_msg.header.stamp = rospy.Time.now()  # Use the current time for the header
                pose_factor_msg.header.frame_id = self.address_to_name[sender_address]  # e.g. "A"
                pose_factor_msg.key1 = key1
                pose_factor_msg.key2 = key1
                pose_factor_msg.pose = prior
                self.pose_factor_pub.publish(pose_factor_msg)
                rospy.loginfo("[%s] Published Prior Factor: %s" % (rospy.Time.now(), key1))
            else:  # Basic message mode
                pass
            # Get the encoded pose data
            prefix = f"relative_pose_{i}_"
            x = getattr(msg, prefix + "x")
            y = getattr(msg, prefix + "y")
            z = getattr(msg, prefix + "z")
            qw = getattr(msg, prefix + "qw")
            qx = getattr(msg, prefix + "qx")
            qy = getattr(msg, prefix + "qy")
            qz = getattr(msg, prefix + "qz")
            sigma_x = getattr(msg, prefix + "sigma_x")
            sigma_y = getattr(msg, prefix + "sigma_y")
            sigma_z = getattr(msg, prefix + "sigma_z")
            sigma_psi = getattr(msg, prefix + "sigma_psi")
            rho_xy = getattr(msg, prefix + "rho_xy")
            rho_xpsi = getattr(msg, prefix + "rho_xpsi")
            rho_ypsi = getattr(msg, prefix + "rho_ypsi")
            encoded_pose = [x, y, z, qw, qx, qy, qz, sigma_x, sigma_y, sigma_z, sigma_psi, rho_xy, rho_xpsi, rho_ypsi]
            # Create a PoseWithCovariance message
            pose_with_covariance = decode_pwc_from_int(encoded_pose, "between")
            # Create a PoseFactorStamped message
            pose_factor_msg = PoseFactorStamped()
            pose_factor_msg.header.stamp = rospy.Time.now()  # Use the current time for the header
            pose_factor_msg.header.frame_id = self.address_to_name[sender_address]  # e.g. "A"
            pose_factor_msg.key1 = key1
            pose_factor_msg.key2 = key2 if key2 is not None else ""
            pose_factor_msg.pose = pose_with_covariance
            # Publish the pose factor message
            self.pose_factor_pub.publish(pose_factor_msg)
            rospy.loginfo("[%s] Published Pose Factor: %s -> %s" % (rospy.Time.now(), key1, key2))
        # Now iterative over the range events and publish them
        for i in range(num_poses): #0,1,2,3
            prefix = f"range_event_{i}_"
            remote_address = getattr(msg, prefix + "remote_address")
            index_or_measured_range = getattr(msg, prefix + "index_or_measured_range")
            sigma_range = getattr(msg, prefix + "sigma_range")
            depth = getattr(msg, prefix + "depth")
            encoded_range_event = [remote_address, index_or_measured_range, sigma_range, depth]
            decoded_range_event = decode_range_event_from_int(encoded_range_event)
            #decoded_range_event = [remote_address, index, measured_range, sigma_range, depth]
            # Check how we should handle the range event
            remote_name = self.address_to_name[decoded_range_event[0]]
            if "L" in remote_name:  # This is a range between the sender and the landmark
                #decoded_range_event = [remote_address, None, measured_range, sigma_range, depth]
                # Build a RangeFactorStamped Directly
                range_factor_msg = RangeFactorStamped()
                range_factor_msg.header.stamp = rospy.Time.now()
                range_factor_msg.header.frame_id = self.address_to_name[sender_address]  # e.g. "A"
                range_factor_msg.key1 = pose_keys[i+1]  # e.g. "A0"
                range_factor_msg.key2 = remote_name  # e.g. "L0"
                range_factor_msg.meas_range = decoded_range_event[2]  # This is the measured range
                range_factor_msg.range_sigma = decoded_range_event[3]  # This is the sigma range
                range_factor_msg.depth1 = decoded_range_event[4]  # Depth of the sender
                range_factor_msg.depth2 = self.landmarks[remote_name][2]  # Depth of the landmark
                rospy.loginfo("[%s] Published Range Factor: %s -> %s, measured_range=%.2f" % (
                    rospy.Time.now(), pose_keys[i+1], remote_name, decoded_range_event[2]))
                self.range_factor_pub.publish(range_factor_msg)
            elif decoded_range_event[1] == None: # This is a range initiated by the sender
                # decoded_range_event = [remote_address, None, measured_range, sigma_range, depth]
                # We should have an entry in partial ranges for this:
                found_partial = False
                assoc_entry = {}
                for pr in self.partial_ranges:
                    if pr["key2"] == pose_keys[i+1]: # We recieved this symbol as payload
                        pr["measured_range"] = decoded_range_event[2]
                        pr["sigma_range"] = decoded_range_event[3]
                        pr["depth2"] = decoded_range_event[4]
                        found_partial = True
                        assoc_entry = pr
                        self.partial_ranges.remove(pr)  # Remove the entry from partial ranges
                        break
                if found_partial:
                    # Now, we can send this RangeFactorStamped message
                    range_factor_msg = RangeFactorStamped()
                    range_factor_msg.header.stamp = rospy.Time.now()
                    range_factor_msg.header.frame_id = self.address_to_name[sender_address]
                    range_factor_msg.key1 = assoc_entry["key1"]  # e.g. "A0"
                    range_factor_msg.key2 = assoc_entry["key2"]
                    range_factor_msg.meas_range = assoc_entry["measured_range"]
                    range_factor_msg.range_sigma = assoc_entry["sigma_range"]
                    range_factor_msg.depth1 = assoc_entry["depth1"]
                    range_factor_msg.depth2 = assoc_entry["depth2"]
                    rospy.loginfo("[%s] Published Range Factor: %s -> %s, measured_range=%.2f" % (
                        rospy.Time.now(), assoc_entry["key2"], assoc_entry["key1"], assoc_entry["measured_range"]))
                    self.range_factor_pub.publish(range_factor_msg)
                elif decoded_range_event[0] == self.local_address:  # This is a range to us
                    # If we don't have one, thats and error and we should log it
                    rospy.logwarn("[%s] No partial range found for %s" % (rospy.Time.now(), pose_keys[i+1]))
                elif decoded_range_event[0] != self.local_address:  # This is a range transmitted from the sender to some other agent
                    # We need to add a partial to the partial ranges
                    self.partial_ranges.append({
                        "timestamp": rospy.Time.now().to_sec(),
                        "key1": pose_keys[i+1],  # e.g. "A0"
                        "key2": None,  # This will be filled later when the msg is received
                        "remote_address": decoded_range_event[0],
                        "index": None,  # This will be filled later when the msg is received
                        "measured_range": decoded_range_event[2],
                        "sigma_range": decoded_range_event[3],
                        "depth1": decoded_range_event[4],
                        "depth2": None # Depth of the other agent
                    })
            elif decoded_range_event[1] is not None:  # This is a range initiated by an agent and recieved by the sender
                transmitted_payload = self.address_to_name[decoded_range_event[0]] + str(int(decoded_range_event[1]))  # e.g. "L0" or "A2"
                # decoded_range_event = [remote_address, index, None, None, depth]
                found_partial = False
                assoc_entry = {}
                # Transmitted payload is of the form "A0"
                # key1 is of the same form as transmitted
                for pr in self.partial_ranges:
                    if pr["key1"] == transmitted_payload: # looking for a match to our key1
                        pr["key2"] = pose_keys[i+1]
                        pr["index"] = decoded_range_event[1]
                        pr["depth2"] = decoded_range_event[4]
                        found_partial = True
                        assoc_entry = pr
                        self.partial_ranges.remove(pr)
                        break
                if found_partial:
                    # Now, we can send this RangeFactorStamped message
                    range_factor_msg = RangeFactorStamped()
                    range_factor_msg.header.stamp = rospy.Time.now()
                    range_factor_msg.header.frame_id = self.address_to_name[sender_address]
                    range_factor_msg.key1 = assoc_entry["key1"]  # e.g. "A0
                    range_factor_msg.key2 = assoc_entry["key2"]
                    range_factor_msg.meas_range = assoc_entry["measured_range"]
                    range_factor_msg.range_sigma = assoc_entry["sigma_range"]
                    range_factor_msg.depth1 = assoc_entry["depth1"]
                    range_factor_msg.depth2 = assoc_entry["depth2"]
                    rospy.loginfo("[%s] Published Range Factor: %s -> %s, measured_range=%.2f" % (
                        rospy.Time.now(), assoc_entry["key1"], assoc_entry["key2"], assoc_entry["measured_range"]))
                    self.range_factor_pub.publish(range_factor_msg)
                elif decoded_range_event[0] == self.local_address:  # This is a range to us
                    # If we don't have one, thats and error and we should log it
                    # Get a list of all key2 in the partial ranges
                    available_key2s = [pr["key2"] for pr in self.partial_ranges if pr["key2"] is not None]
                    rospy.logwarn("[%s] No partial range found for %s. Unpaired keys in partial dict: %s" % (rospy.Time.now(), pose_keys[i+1], available_key2s))
                elif decoded_range_event[0] != self.local_address:  # This is a range transmitted from the sender to some other agent
                    # We need to add a partial to the partial ranges
                    self.partial_ranges.append({
                        "timestamp": rospy.Time.now().to_sec(),
                        "key1": transmitted_payload,  # e.g. "L0" or "A2"
                        "key2": pose_keys[i+1],  # e.g. "A0"
                        "remote_address": decoded_range_event[0],
                        "index": decoded_range_event[1],
                        "measured_range": None,
                        "sigma_range": None,
                        "depth1": None,
                        "depth2": decoded_range_event[4]  # Depth of the other agent
                    })
                else:
                    rospy.logwarn("[%s] Abnormal address, no partial range found %s" % (rospy.Time.now(), pose_keys[i+1]))
        # Log the test data
        rospy.loginfo("[%s] Received Graph Update" % (rospy.Time.now()))
        return

    def send_ping(self, target_addr):
        """This function sends a ping to the modem
        Args:
            target_addr (int): the target address
            symbol (str): the local key "A1" to put in the payload
        """
        # Remove the target address from the planned target
        if target_addr not in self.planned_targets:
            rospy.logwarn("[%s] Target address %s not in planned targets %s" % (rospy.Time.now(), self.address_to_name[target_addr], self.planned_targets))
            return
        else:
            self.planned_targets.remove(target_addr)
        # Check if we have time to ping before we are inactive
        if self.tdma_status.remaining_active_seconds < self.ping_timeout:
            rospy.logwarn("[%s] Aborting ping to %s, remaining active seconds: %.2f timeout: %.2f" % (rospy.Time.now(),
                self.address_to_name[target_addr], self.tdma_status.remaining_active_seconds, self.ping_timeout))
            self.planned_targets.clear()
            return
        else:
            pass
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
            rospy.loginfo("[%s] Sending Ping to %s with payload %s" % (rospy.Time.now(), self.address_to_name[target_addr], symbol))
            self.cycle_status["pings_attempted"] += 1
            ping_resp = self.ping_client(ping_req)
            # Check if the ping timed out
            if ping_resp.timed_out:
                rospy.logwarn("[%s] Ping to %s Timed Out" % (rospy.Time.now(), self.address_to_name[target_addr]))
                # Ping the next target in the list
                next_tgt = self.planned_targets[0] if self.planned_targets else None
                if next_tgt is not None:
                    rospy.loginfo("[%s] Attempting next target: %s" % (rospy.Time.now(), self.address_to_name[next_tgt]))
                    self.send_ping(next_tgt)
                elif self.send_data:
                    rospy.sleep(1)  # To account for formulating ping data
                    if self.message_mode == "basic":
                        self.send_basic_graph_update()
                    else:
                        self.send_adv_graph_update()
                else:
                    rospy.loginfo("[%s] No more targets to ping" % rospy.Time.now())
                #self.tx_range_data.append([None, rospy.Time.now().to_sec()-self.ping_timeout/2, rospy.Time.now().to_sec(), self.local_address, target_addr, None, None, None, None, None, None])
                return
            else:
                #rospy.loginfo("[%s] Ping Successful: "% (rospy.Time.now()))
                dest = ping_resp.cst.src
                src = ping_resp.cst.dest
                owtt = ping_resp.one_way_travel_time
                tat = ping_resp.tat
                measured_range = owtt * self.sound_speed
                # Convert the timestamp to a ROS Time object
                # NOTE: The timestamp is in float seconds, so we can convert it to a ROS Time object
                timestamp_ns = ping_resp.cst.toa - rospy.Duration.from_sec(owtt)
                timestamp_sec = rospy.Time.from_sec(timestamp_ns.to_sec())

                # Log all the fields
                rospy.loginfo("[%s] Ping Complete %s to %s: timestamp=%s, owtt=%.4f, tat= %.4f, measured_range=%.2f" % (
                    rospy.Time.now(), self.address_to_name[src], self.address_to_name[dest], timestamp_sec, owtt, tat, measured_range))
                # self.tx_range_data.append([timestamp_sec.to_sec(), src, dest, owtt, measured_range])
                # Request preintegration
                self.request_preintegration(timestamp_ns, True) # Request a relative pose measurement (and advance the pose index)
                if "L" in self.address_to_name[dest]:  #Send the range factor message right away
                    range_factor_msg = RangeFactorStamped()
                    range_factor_msg.header.stamp = timestamp_sec
                    range_factor_msg.header.frame_id = "modem"
                    range_factor_msg.key1 = symbol
                    range_factor_msg.key2 = self.address_to_name[dest]  # e.g. "L0" or "A2"
                    range_factor_msg.meas_range = measured_range
                    range_factor_msg.range_sigma = self.range_sigma
                    range_factor_msg.depth1 = self.depth
                    range_factor_msg.depth2 = self.landmarks[self.address_to_name[dest]][2]
                    self.range_factor_pub.publish(range_factor_msg)
                    rospy.loginfo("[%s] Published Range Factor: %s -> %s, measured_range=%.2f" % (
                        timestamp_sec, symbol, self.address_to_name[dest], measured_range))
                else:
                    self.partial_ranges.append({
                        "timestamp": timestamp_sec.to_sec(),
                        "key1": symbol,
                        "key2": self.address_to_name[dest],  # e.g. "L0" or "A2"
                        "remote_address": dest,
                        "index": None,  # This will be filled later when the msg is received
                        "measured_range": measured_range,  # This will be filled later when the msg is received
                        "sigma_range": self.range_sigma,    # This will be filled later when the msg is received
                        "depth1": self.depth,
                        "depth2": None  # This will be filled later when the msg is received
                    })
                next_tgt = self.planned_targets[0] if self.planned_targets else None
                if next_tgt is not None:
                    rospy.loginfo("[%s] Attempting next target: %s" % (rospy.Time.now(), self.address_to_name[next_tgt]))
                    self.send_ping(next_tgt)
                elif self.send_data:
                    rospy.sleep(1)  # To account for formulating ping data
                    if self.message_mode == "basic":
                        self.send_basic_graph_update()
                    else:
                        self.send_adv_graph_update()
                else:
                    rospy.loginfo("[%s] No more targets to ping" % rospy.Time.now())
                return
        except rospy.ServiceException as e:
            rospy.logerr("[%s] Ping Service Call Failed: %s" % (rospy.Time.now(), e))
        return

    def send_basic_graph_update(self):
        """This function sends test data to the modem
        """
        # Create a test data message
        active_message = self.graph_update_msg
        self.graph_update_pub.publish(active_message)
        self.graph_update_msg = BasicGraphUpdate()  # Reset the message for the next cycle
        self.graph_update_msg.sender_address = int(self.local_address)
        rospy.loginfo("[%s] Sent Basic Graph Update" % (rospy.Time.now()))
        self.acomms_event_pub.publish("priority=2,pattern=([0.0.0.255]:0.5)([0.255.0.50]:2.0),cycles=1")
        return

    def send_adv_graph_update(self):
        """This function sends test data to the modem
        """
        # Find the key of the first pose in the graph update message
        addresss = self.graph_update_msg.sender_address
        index = self.graph_update_msg.first_key_index
        symbol_of_prior = chr(ord("A") + addresss) + str(index)
        #TODO: Call to get the prior value and marginal from estimator
        prior_pose = PoseWithCovarianceStamped(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id="modem"
            ),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                ),
                covariance=[0.0] * 36  # Placeholder covariance
            )
        )
        # Add Pose Prior to the graph update message
        self.add_prior_to_graph_update(prior_pose)
        active_message = self.graph_update_msg
        self.graph_update_pub.publish(active_message)
        self.graph_update_msg = AdvancedGraphUpdate()  # Reset the message for the next cycle
        self.graph_update_msg.sender_address = int(self.local_address)
        rospy.loginfo("[%s] Sent Advanced Graph Update" % (rospy.Time.now()))
        self.acomms_event_pub.publish("priority=2,pattern=([0.0.0.255]:0.5)([0.255.0.50]:2.0),cycles=1")
        return

    # Sensor Callbacks:s
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
        key2_index = key1_index + 1
        key1 = local_chr + str(key1_index)
        key2 = local_chr + str(key1_index+1)
        ti = self.pose_time_lookup[local_chr + str(key1_index)]
        try:
            rospy.loginfo(f"Attempting to preintegrate between {ti} and {tj}")
            
            response = self.preintegrate_imu(ti, tj)
            rospy.loginfo(f"Received preintegrated pose between {ti} and {tj}")
            if adv_pose:
                x_ij = response.pose_delta
                # pose_delta is a PoseWithCovarianceStamped message
                position = np.array([
                    x_ij.pose.pose.position.x,
                    x_ij.pose.pose.position.y,
                    x_ij.pose.pose.position.z
                ])
                orientation = np.array([
                    x_ij.pose.pose.orientation.x,
                    x_ij.pose.pose.orientation.y,
                    x_ij.pose.pose.orientation.z,
                    x_ij.pose.pose.orientation.w
                ])
                # Convert the covariance to a numpy array and reshape it to 6x6
                covariance = np.array(x_ij.pose.covariance).reshape((6, 6))
                sigmas = np.sqrt(np.diag(covariance))
            else:
                # This allows for calling preintegration to clear the queue without advancing the pose
                pass

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            response = PreintegrateImuResponse(
                pose_delta=PoseWithCovarianceStamped(
                    header=Header(
                        stamp=tj,
                        frame_id="imu"
                    ),
                    pose=Pose(
                        position=Point(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    covariance=[1.0] * 36  # Placeholder covariance
                )
            )
        # Add the pose_delta (a PoseWithCovarianceStamped message) to the graph_update message
        self.add_pwcs_to_graph_update(key1_index, response.pose_delta)
        # Log the pose factor
        pose_factor_msg = PoseFactorStamped()
        pose_factor_msg.header.stamp = tj
        pose_factor_msg.header.frame_id = "modem"
        pose_factor_msg.key1 = key1
        pose_factor_msg.key2 = key2
        # pose PoseWithCovariance
        pose_factor_msg.pose.pose.position.x = response.pose_delta.pose.pose.position.x
        pose_factor_msg.pose.pose.position.y = response.pose_delta.pose.pose.position.y
        pose_factor_msg.pose.pose.position.z = response.pose_delta.pose.pose.position.z
        pose_factor_msg.pose.pose.orientation.x = response.pose_delta.pose.pose.orientation.x
        pose_factor_msg.pose.pose.orientation.y = response.pose_delta.pose.pose.orientation.y
        pose_factor_msg.pose.pose.orientation.z = response.pose_delta.pose.pose.orientation.z
        pose_factor_msg.pose.pose.orientation.w = response.pose_delta.pose.pose.orientation.w
        # pose sigmas
        pose_factor_msg.pose.covariance = np.array(response.pose_delta.pose.covariance).reshape((6, 6)).flatten().tolist()
        self.pose_factor_pub.publish(pose_factor_msg)
        # Advance the key indices and the time
        self.modem_addresses[local_chr][1] = key2_index
        self.pose_time_lookup[key2] = tj
        return

    def add_prior_to_graph_update(self, pose: PoseWithCovarianceStamped):
        """This function adds a PoseWithCovarianceStamped message to the graph update message
        Args:
            pose (PoseWithCovarianceStamped): The pose to add to the graph update
            key_index (int): The index of the key in the pose factor
        """
        # Encode the PoseWithCovarianceStamped message as integer values
        encoded_pose = encode_pwcs_as_int(pose, "prior")
        # Add the encoded pose to the graph update message
        prefix = "pose_prior_0_"
        setattr(self.graph_update_msg, prefix + "x", encoded_pose[0])
        setattr(self.graph_update_msg, prefix + "y", encoded_pose[1])
        setattr(self.graph_update_msg, prefix + "z", encoded_pose[2])
        setattr(self.graph_update_msg, prefix + "qw", encoded_pose[3])
        setattr(self.graph_update_msg, prefix + "qx", encoded_pose[4])
        setattr(self.graph_update_msg, prefix + "qy", encoded_pose[5])
        setattr(self.graph_update_msg, prefix + "qz", encoded_pose[6])
        setattr(self.graph_update_msg, prefix + "sigma_x", encoded_pose[7])
        setattr(self.graph_update_msg, prefix + "sigma_y", encoded_pose[8])
        setattr(self.graph_update_msg, prefix + "sigma_z", encoded_pose[9])
        setattr(self.graph_update_msg, prefix + "sigma_psi", encoded_pose[10])
        # Log the addition of the pose
        rospy.loginfo("[%s] Added PoseWithCovarianceStamped %s to Graph Update)" % (rospy.Time.now(), prefix))
        return

    def add_pwcs_to_graph_update(self, key1_index: int, preintegratded_pose: PoseWithCovarianceStamped):
        """This function adds a PoseWithCovarianceStamped message to the graph update message
        Args:
            preintegratded_pose (PoseWithCovarianceStamped): The pose to add to the graph update
            key1_index (int): The index of the first key in the pose factor
        """
        # Encode the PoseWithCovarianceStamped message as integer values
        encoded_pose = encode_pwcs_as_int(preintegratded_pose, "between")
        # Get the number of poses currently stored:
        num_poses = self.graph_update_msg.num_poses
        prefix = f"relative_pose_{num_poses}_"
        # If this is the first pose, update the first_key_index
        if num_poses == 0:
            self.graph_update_msg.first_key_index = key1_index
        # Add the encoded pose to the graph update message
        setattr(self.graph_update_msg, prefix + "x", encoded_pose[0])
        setattr(self.graph_update_msg, prefix + "y", encoded_pose[1])
        setattr(self.graph_update_msg, prefix + "z", encoded_pose[2])
        setattr(self.graph_update_msg, prefix + "qw", encoded_pose[3])
        setattr(self.graph_update_msg, prefix + "qx", encoded_pose[4])
        setattr(self.graph_update_msg, prefix + "qy", encoded_pose[5])
        setattr(self.graph_update_msg, prefix + "qz", encoded_pose[6])
        setattr(self.graph_update_msg, prefix + "sigma_x", encoded_pose[7])
        setattr(self.graph_update_msg, prefix + "sigma_y", encoded_pose[8])
        setattr(self.graph_update_msg, prefix + "sigma_z", encoded_pose[9])
        setattr(self.graph_update_msg, prefix + "sigma_psi", encoded_pose[10])
        if self.message_mode == "basic":
            # For basic mode, we don't need the rho values
            setattr(self.graph_update_msg, prefix + "rho_xy", 0)
            setattr(self.graph_update_msg, prefix + "rho_xpsi", 0)
            setattr(self.graph_update_msg, prefix + "rho_ypsi", 0)
        else:
            pass # Adv version does not include these values
        # Increment the number of poses in the graph update message
        self.graph_update_msg.num_poses += 1
        # Log the addition of the pose
        rospy.loginfo("[%s] Added PoseWithCovarianceStamped %s to Graph Update)" % (rospy.Time.now(), prefix))
        return

    def add_range_event_to_graph_update(self, remote_address: int, remote_index: int, measured_range: float, sigma_range: float = 1.0):
        #[remote_address, index_or_measured_range, sigma_range, depth]
        """This function adds a range measurement to the graph update message"""
        # Encode the range measurement as integer values
        depth = self.depth
        encoded_range = encode_range_event_as_int(remote_address, remote_index, measured_range, sigma_range, depth)
        # Get the number of ranges currently stored (a proxy is the number of depth readings which are not zero)
        num_ranges = 0
        for i in range(4):
            try:
                depth_val = getattr(self.graph_update_msg, f"range_event_{i}_depth")
                if depth_val > 0.0:
                    num_ranges += 1
                else:
                    break  # First unused slot
            except AttributeError:
                break  # Field does not exist
        if num_ranges >= 4:
            rospy.logwarn("Maximum number of range events already added to graph update!")
            return  # Optionally raise exception or overwrite last one
        prefix = f"range_event_{num_ranges}_"
        # Update the fields at this index
        setattr(self.graph_update_msg, prefix + "remote_address", encoded_range[0])
        setattr(self.graph_update_msg, prefix + "index_or_measured_range", encoded_range[1])
        setattr(self.graph_update_msg, prefix + "sigma_range", encoded_range[2])
        setattr(self.graph_update_msg, prefix + "depth", encoded_range[3])
        rospy.loginfo("[%s] Added Range %s to Graph Update)" % (rospy.Time.now(), prefix))
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
        # Log the reciept (disabled for dockside)
        #rospy.loginfo("[%s] Received GPS data" % pose_time)
        return

    def on_in_water(self, msg: Bool):
        """This function receives the in water status from the estimator
        Args:
            msg (Bool): The in water status
        """
        self.in_water = msg.data
        if msg.data == True and not self.in_water:
            rospy.loginfo("[%s] Vehicle has entered the water" % rospy.Time.now())
            # Set pose index to 0
            local_chr = chr(ord("A") + self.local_address)
            self.modem_addresses[local_chr][1] = 0
            self.pose_time_lookup[local_chr + str(0)] = rospy.Time.now()
            # Request preintegration to clear the queue, but don't advance the pose index
            #sself.request_preintegration(rospy.Time.now(), adv_pose=False)
        elif msg.data == False and self.in_water:
            rospy.loginfo("[%s] Vehicle has left the water" % rospy.Time.now())
        else:
            rospy.loginfo("[%s] Vehicle is out of water" % rospy.Time.now())
        return

    def on_sound_speed(self, msg: Bar30SoundSpeed):
        """This function receives the sound speed data from the Bar30 sensor
        Args:
            msg (Bar30SoundSpeed): The sound speed data
        """
        self.bar30_sound_speed = msg.sound_speed
        # Report the difference between the Bar30 sound speed and the configured sound speed
        if abs(self.bar30_sound_speed-self.sound_speed)/self.sound_speed > 0.1:
            rospy.logwarn("[%s] Mismatched C: Bar30 Sound Speed: %.2f m/s, Configured Sound Speed: %.2f m/s" % (rospy.Time.now(), self.bar30_sound_speed, self.sound_speed))
        return

    def on_nav_state(self, msg: PoseStamped):
        """This function receives the navigation state from the estimator
        Args:
            msg (PoseStamped): The navigation state data
        """
        # Get the current time
        pose_time = msg.header.stamp
        self.depth = msg.pose.position.z  # Update the depth of the vehicle
        return

    # Logging Functions:
    def on_xst(self, msg: XST):
        """This function receives the XST data from the modem
        Args:
            msg (XST): The XST data from the modem
        """
        # Get timestamp from header
        xst_time = msg.time.to_sec()
        src = msg.src
        dest = msg.dest
        return

    def on_cst(self, msg: CST):
        """ This function only logs CST data if we are listening (i.e. not active)"""
        # Get timestamp from header
        cst_time = rospy.Time(msg.toa.secs, msg.toa.nsecs).to_sec()
        src = msg.src
        dest = msg.dest
        dop = msg.dop
        stddev_noise = msg.stddev_noise
        snr_in = msg.snr_in
        snr_out = msg.snr_out
        num_frames = msg.num_frames
        return

    def on_range_log(self, msg: PingReply):
        """This function logs the range data from the modem
        Args:
            msg (PingReply): The range data from the modem
        """
        # Get timestamp from header
        message_timestamp = msg.header.stamp.to_sec()
        range_timestamp = rospy.Time(msg.cst.toa.secs, msg.cst.toa.nsecs).to_sec()
        src = msg.dest #NOTE: inverted due to ping reply
        dest = msg.src
        owtt = msg.owtt
        measured_range = owtt * self.sound_speed if owtt is not None else None
        # Update the cycle status
        # rospy.loginfo("[%s] rnage timestamp: %.4f, message timestamp: %.4f, last range timestamp: %.4f, src: %s" % (
        #     rospy.Time.now(), range_timestamp, message_timestamp, self.cycle_status["last_range_timestamp"].to_sec(), chr(ord("A") + src)))
        self.cycle_status["pings_successful"] += 1
        self.cycle_status["last_range_interval"] = rospy.Duration.from_sec(
            range_timestamp - self.cycle_status["last_range_timestamp"].to_sec()
        )
        self.cycle_status["last_range_timestamp"] = rospy.Time.from_sec(range_timestamp)
        self.cycle_status["last_range_target"] = dest
        self.cycle_status["last_range_distance"] = np.round(measured_range,4)
        if measured_range > self.cycle_status["max_range"]:
            self.cycle_status["max_range"] = np.round(measured_range,4)
        elif measured_range < self.cycle_status["min_range"]:
            self.cycle_status["min_range"] = np.round(measured_range,4)
        else:
            pass
        # Create a ReceivedSignalStats message
        received_signal_stats = ReceivedSignalStats()
        received_signal_stats.header.stamp = rospy.Time.now()
        received_signal_stats.header.frame_id = "modem"
        received_signal_stats.toa = rospy.Time.from_sec(range_timestamp)
        received_signal_stats.src = msg.src
        received_signal_stats.dest = msg.dest
        received_signal_stats.signal_type = "range"
        received_signal_stats.carrier = msg.cst.carrier
        received_signal_stats.bandwidth = msg.cst.bandwidth
        received_signal_stats.rate_num = msg.cst.rate_num
        # Get packet type from CST
        packet_types = ["error", "FSK", "FSK_mini", "PSK", "PSK_mini", "PSK_FDP"]
        if msg.cst.packet_type == -1:
            received_signal_stats.packet_type = "unknown"
        else:
            received_signal_stats.packet_type = packet_types[msg.cst.packet_type] if msg.cst.packet_type < len(packet_types) else "unknown"
        received_signal_stats.mfd_peak = msg.cst.mfd_peak
        received_signal_stats.mfd_pow = msg.cst.mfd_pow
        received_signal_stats.mfd_ratio = msg.cst.mfd_ratio
        received_signal_stats.mfd_spl = msg.cst.mfd_spl
        received_signal_stats.snr_rss = msg.cst.snr_rss
        received_signal_stats.snr_in = msg.cst.snr_in
        received_signal_stats.snr_out = msg.cst.snr_out
        received_signal_stats.mse = msg.cst.mse
        received_signal_stats.dop = msg.cst.dop
        received_signal_stats.noise = msg.cst.noise
        received_signal_stats.tx_level = msg.tx_level
        received_signal_stats.owtt = msg.owtt
        received_signal_stats.tat = msg.tat
        self.recieved_signal_stats_pub.publish(received_signal_stats)
        return

    def send_acomms_status(self, event):
        """This function sends the ACOMMS status to the modem"""
        # Create the ACOMMS event message
        acomms_cycle_status = AcommsCycleStatus()
        acomms_cycle_status.header.stamp = rospy.Time.now()
        acomms_cycle_status.header.frame_id = "modem"
        acomms_cycle_status.pings_attempted = self.cycle_status["pings_attempted"]
        acomms_cycle_status.pings_successful = self.cycle_status["pings_successful"]
        acomms_cycle_status.pings_pct_successful = np.round(100*self.cycle_status["pings_successful"]/self.cycle_status["pings_attempted"], 2) if self.cycle_status["pings_attempted"] > 0 else 0.0
        acomms_cycle_status.min_range = self.cycle_status["min_range"]
        acomms_cycle_status.max_range = self.cycle_status["max_range"]
        acomms_cycle_status.last_range_timestamp = self.cycle_status["last_range_timestamp"]
        acomms_cycle_status.last_range_interval = self.cycle_status["last_range_interval"]
        acomms_cycle_status.last_range_target = self.cycle_status["last_range_target"]
        acomms_cycle_status.last_range_distance = self.cycle_status["last_range_distance"]
        self.cycle_status_pub.publish(acomms_cycle_status)
        rospy.loginfo("[%s] Cycle Status sent" % rospy.Time.now())
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