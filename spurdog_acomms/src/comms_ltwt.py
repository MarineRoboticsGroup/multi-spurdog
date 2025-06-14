#!/usr/bin/env python3
import rospy
import csv
from os.path import dirname, join, abspath
import numpy as np
from datetime import datetime
import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance, PoseWithCovarianceStamped
from ros_acomms_msgs.msg import(
    TdmaStatus, QueueStatus
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)
from spurdog_acomms.msg import(
    InitPrior, PartialGraph, CommsCycleStatus
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
    parse_nmea_cacst,
    parse_nmea_caxst,
    parse_nmea_carev,
    parse_nmea_catxf
)

class CycleManager:
    """This is a node to run the comms cycle for the vehicle.
        - This is a lightweight version to test simple message reciept rates"""
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.local_address = int(rospy.get_param("modem_address", 0))
        self.num_agents = int(rospy.get_param("num_agents", 1))
        self.num_landmarks = int(rospy.get_param("num_landmarks", 2))
        self.landmarks = {"L0":[-74.5193539608157,-38.9298973079931,1.5], "L1":[66.5150726324041,25.969767675496275,1.5]} # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = float(rospy.get_param("sound_speed", 1486))
        # Variables for addressing
        self.modem_addresses = {}
        self.cycle_target_mapping = {}
        # Variables for tdma and cycle execution
        self.tdma_status = TdmaStatus()
        self.tdma_cycle_sequence = 0
        self.queue_status = QueueStatus()
        #self.msg_preload = 5 # seconds to allow for encoding the message before the tdma window opens
        # Variables for acomms event topic
        self.ping_method = "ping with payload"
        self.ping_timeout = 5 # seconds
        self.message_on_safe = False # True if messages not allowed
        #self.ping_on_safe = True # True if pings not allowed
        self.data_sent = False
        self.ping_slot_1_attempted = False # True if the first ping slot has been attempted
        self.ping_slot_2_attempted = False # True if the second ping slot has been attempted
        # Variables for external sensors
        self.gps_fix = [[1,2,3],[0,0,0,1],[1.7,1.7,3.5,0.1,0.1,0.1]] # [position, orientation, covariance]
        # Variables for message handling
        self.staged_init_prior = InitPrior()
        self.staged_partial_graph = PartialGraph()
        self.range_data = []
        self.cst_data = []
        self.xst_data = []
        self.preintegration_data = []
        #self.partial_graph_data = {}
        #self.cycle_graph_data = {}
        #self.inbound_init_priors = {}
        self.inbound_partial_graphs = {}
        self.pose_time_lookup = {}
        self.in_water = False
        self.smooth_poses = False
        self.init_complete = True
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
        self.preintegrate_imu = rospy.ServiceProxy("preintegrate_imu", PreintegrateImu)
        rospy.loginfo("[%s] Services ready, initializing topics" % rospy.Time.now())
        # Initialize topics
        self.tdma_from_modem = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_status, queue_size=1)
        self.queue_status_from_modem = rospy.Subscriber("modem/queue_status", QueueStatus, self.on_queue_status, queue_size=1)
        rospy.loginfo("[%s] TDMA Status msg registered Configured" % rospy.Time.now())
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        # Establish the message subs and pubs
        self.init_prior_pub = rospy.Publisher("modem/to_acomms/init_prior", InitPrior, queue_size=1)
        self.partial_graph_pub = rospy.Publisher("modem/to_acomms/partial_graph", PartialGraph, queue_size=1)
        self.acomms_event_pub = rospy.Publisher("led_command", String, queue_size=1)
        # Initialize Subscribers for handling external sensors
        self.gps = rospy.Subscriber("gps", PoseWithCovarianceStamped, self.on_gps)
        self.mission_status = rospy.Subscriber("mission_state", String, self.on_mission_state)
        self.comms_cycle_status = rospy.Subscriber("comms_cycle_status", CommsCycleStatus, self.on_comms_cycle_status)
        # Initialize the modem addresses and cycle targets
        rospy.loginfo("[%s] Topics ready, initializing comms cycle" % rospy.Time.now())
        self.configure_comms_cycle()
        # Start the cycle
        rospy.loginfo("[%s] Comms Cycle Configured" % rospy.Time.now())
        rospy.sleep(10) # allow for modem to configure

    def configure_comms_cycle(self):
        """This function configures the comms cycle for the vehicle.
        """
        # Get the modem addresses and cycle targets
        self.modem_addresses = configure_modem_addresses(self.num_agents, self.num_landmarks, self.local_address)
        self.cycle_target_mapping = configure_cycle_targets(self.modem_addresses)
        # Confifure the first pose in pose_time_lookup
        local_chr = chr(ord("A") + self.local_address)
        self.pose_time_lookup[local_chr + str(0)] = rospy.Time.now()  # Initial pose at time of cycle start
        # Print cycle targets for debugging
        rospy.loginfo("[%s] Cycle Targets: %s" % (rospy.Time.now(), self.cycle_target_mapping))
        return

    def on_comms_cycle_status(self, msg):
        """This function handles the comms cycle status message
        Args:
            msg (CommsCycleStatus): The comms cycle status message
        """
        # Get message field:
        cycle_seq = msg.sequence_number
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
        msg_id = msg.header.seq
        msg_stamp = msg.header.stamp.to_nsec()
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
        elif current_slot == 1 and current_slot != self.tdma_status.current_slot:
            self.partial_graph_pub.publish(self.staged_partial_graph)
        elif we_are_active and remaining_sec_in_slot < 2.1*self.ping_timeout:
            # Check if we need to manually recover the cycle:
            if self.data_sent == False:
                self.data_sent = True
                self.poke_ping_cycle(10)
            else:
                pass
        else:
            # This is an escape to prevent the ping cycle from executing if we have no other agents or landmarks
            pass

        # Regardless, update TDMA
        self.tdma_status = msg
        return

    def on_tdma_cycle_reset(self):
        """This function is called when the TDMA slot returns to 0
        """
        self.ping_slot_1_attempted = False
        self.ping_slot_2_attempted = False
        self.data_sent = False
        if self.init_complete:
            # NOTE: The smoothed relative pose is incorporated within build_partial_graph
            self.staged_partial_graph = self.build_partial_graph()
            # Publish the partial graph
            if not self.message_on_safe:
                self.partial_graph_pub.publish(self.staged_partial_graph)
            # NOTE: If the graph fails the check, this will be an empty message
            # If there are no successful pings, the graph will be empty and preintegration won't occur
            # The checking function evaluates if the graph is connected, and without duplicates
            # It also check sfor gross errors (like violating the size limit)
            self.tdma_cycle_sequence += 1
        elif self.staged_init_prior == InitPrior():
            # If we are not initialized, we need to build the init prior
            self.staged_init_prior = self.build_init_prior()
            # Publish the init prior
            if not self.message_on_safe:
                self.init_prior_pub.publish(self.staged_init_prior)
                self.init_complete = True
                self.tdma_cycle_sequence = 0
        else:
            # We are initialized, but others are not, so we need to reset the cycle
            self.tdma_cycle_sequence = 0
            rospy.logwarn("[%s] TDMA Cycle Reset, waiting for init prior data" % rospy.Time.now())
        return

    def on_queue_status(self, msg):
        """This function handles the queue status message from the modem
        Args:
            msg (QueueStatus): The queue status message
        """
        # Get the queue status
        msg_id = msg.header.seq
        msg_stamp = msg.header.stamp.to_nsec()
        msg_count = msg.total_message_count
        # if msg_count > 0 or self.tdma_status.we_are_active == False:
        #     self.ping_on_safe = True  # If there are messages in the queue, we should not ping
        #     #rospy.loginfo("[%s] Queue Status: %d messages in queue" % (rospy.Time.now(), msg_count))
        #     if msg_count > 1:
        #         rospy.logwarn("[%s] Queue Status: %d messages in queue" % (rospy.Time.now(), msg_count))
        # if self.queue_status.total_message_count > 0 and msg_count == 0:
        #     # We have flushed the queue and should now allow ping
        #     rospy.sleep(3)
        #     self.ping_on_safe = False
        #     self.ping_slot_1_attempted = False
        #     self.ping_slot_2_attempted = False
        #     time_to_clear_queue = self.tdma_status.slot_duration_seconds - self.tdma_status.remaining_slot_seconds
        #     # Report the time to clear the queue to 2 decimal places
        #     rospy.loginfo("[%s] Queue Cleared, Time to Clear: %.2f seconds" % (rospy.Time.now(), time_to_clear_queue))
        # elif self.tdma_status.we_are_active == False and msg_count == 0:
        #     # If we are not active and there are messages in the queue, we should not ping
        #     self.ping_on_safe = False
        # else:
        #     pass
        self.queue_status = msg
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
            # Convert time (ROS Time in sec) to ROS Time in nsec
            rcvd_stamp = rospy.Time.from_sec(recieved_ping_time)
            if data[1] == "PNG" and dest == self.local_address:
                self.request_preintegration(rcvd_stamp, True) # Request a relative pose measurement
                rospy.loginfo("[%s] Received Ping from %s" % (recieved_ping_time, chr(ord("A") + src)))
                self.acomms_event_pub.publish("priority=2,pattern=([0.0.255.0]:1.0),cycles=1")
            elif data[1] == "PNG":
                rospy.loginfo("[%s] Overheard Ping from %s to %s" % (recieved_ping_time, chr(ord("A") + src), chr(ord("A") + dest)))
            else:
                rospy.logerr("[%s] Received $CACMA with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMR": # Modem-to-host acknowledgement of a ping response
            src, dest, recieved_ping_time, owtt = parse_nmea_cacmr(data)
            if data[0] == "PNR" and src == self.local_address:
                rospy.loginfo("[%s] Received Ping Response from %s" % (recieved_ping_time, chr(ord("A") + dest)))
                self.acomms_event_pub.publish("priority=2,pattern=([0.255.0.0]:1.0),cycles=1")
            elif data[0] == "PNR":
                rospy.loginfo("[%s] Overheard Ping Response from %s to %s" % (recieved_ping_time, chr(ord("A") + src), chr(ord("A") + dest)))
            else:
                rospy.logerr("[%s] Received $CACMR with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CARFP" and data[5] == "-1": # Modem-to-host acknowledgement of a minipacket ping payload
            src, dest, recieved_msg_time, num_frames, payload = parse_nmea_carfp(data)
            if recieved_msg_time == None or not src == None or not dest == None:
                rospy.logerr("[%s] CARFP message is missing required fields" % rospy.Time.now())
                return
            elif dest == self.local_address:
                rospy.loginfo("[%s] Received Ping from %s with payload %s" % (recieved_msg_time, chr(ord("A") + src), payload))
            elif dest != self.local_address:
                rospy.logerr("[%s] Overheard Ping-related $CARFP from %s to %s with paylaod %s" % (recieved_msg_time, chr(ord("A") + src), chr(ord("A") + dest), payload))
            else:
                rospy.logerr("[%s] Received $CARFP with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACST": # Modem-to-host report of signal recieved
            cst_statistics = parse_nmea_cacst(data)
            self.cst_data.append(cst_statistics)

        elif nmea_type == "$CAXST": # Modem-to-host report of signal transmitted
            xst_statistics = parse_nmea_caxst(data)
            self.xst_data.append(xst_statistics)

        elif nmea_type == "$CAREV" and self.ping_method == None: # Modem-to-host $CAREV message to determine the firmware version
            firmware_version = parse_nmea_carev(data)
            if firmware_version[0] == "3":
                # New deckbox firmware
                self.ping_method = "ping with payload"
            else:
                # Old deckbox firmware
                self.ping_method = "no payload"
        elif nmea_type == "$CATXP": # Modem-to-host report of beginning transmission
            # This is a transmit report, we can ignore it for now
            #rospy.loginfo("[%s] Received $CATXP message: %s" % (rospy.Time.now(), data))
            pass
        elif nmea_type == "$CATXF": # Modem-to-host report of end of transmission
            nbytes = parse_nmea_catxf(data)
            self.poke_ping_cycle(nbytes)
            if nbytes > 2:
                self.acomms_event_pub.publish("priority=2,pattern=([255.255.0.0]:1.0),cycles=3")
                pass
        else:
            return
        return

    def poke_ping_cycle(self, nbytes):
        """ This function detects a completed transmission, then checks the type and queues the next transmission"""
        # Check if we are active
        if not self.tdma_status.we_are_active:
            # This handles ping replies
            rospy.logwarn("[%s] Not active, ignoring poke" % rospy.Time.now())
            return
        else:
            first_tgt, second_tgt = self.cycle_target_mapping[self.tdma_status.current_slot]
            # This handles our directed transmissions:
            if nbytes > 10:
                # This is a data transmission
                rospy.loginfo("[%s] Completed data transmission of %d bytes" % (rospy.Time.now(), nbytes))
                # Check the status of the queue
                if self.queue_status.total_message_count > 0:
                    # If there are messages in the queue, we should not ping
                    rospy.logwarn("[%s] Queue Status: %d messages in queue, not pinging" % (rospy.Time.now(), self.queue_status.total_message_count))
                    return
                else:
                    self.data_sent = True
            if self.data_sent == True:
                if self.tdma_status.remaining_slot_seconds > self.ping_timeout:
                    # Check with ping we should attempt
                    if self.ping_slot_1_attempted == False:
                        # Sleep for a half second
                        rospy.sleep(0.5)
                        self.send_ping(first_tgt)
                    elif self.ping_slot_2_attempted == False:
                        self.send_ping(second_tgt)
                    else:
                        # Build and publish the partial graph
                        self.build_partial_graph()
            else:
                # This is an error, we should not have a transmission of 0 bytes
                rospy.logerr("[%s] Transmission of 0 bytes detected, this is an error" % rospy.Time.now())

    def send_ping(self, target_addr):
        """This function sends a ping to the modem
        Args:
            target_addr (int): the target address
            symbol (str): the local key "A1" to put in the payload
        """
        first_tgt, second_tgt = self.cycle_target_mapping[self.tdma_status.current_slot]
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
            # Response
            # bool timed_out
            # float32 one_way_travel_time
            # float32 tat

            # int8 txlevel
            # int8 timestamp_resolution
            # int8 toa_mode
            # int8 snv_on
            # uint32 timestamp
            # Check if the ping timed out
            if ping_resp.timed_out:
                rospy.logwarn("[%s] Ping to %s Timed Out" % (rospy.Time.now(), target_addr))
                if target_addr == first_tgt:
                    self.ping_slot_1_attempted = True
                elif target_addr == second_tgt:
                    self.ping_slot_2_attempted = True
                self.poke_ping_cycle(3)
                self.range_data.append([rospy.Time.now().to_sec() -self.ping_timeout/2, self.local_address, target_addr, None, None])
                return
            else:
                rospy.loginfo("[%s] Ping Successful: "% (rospy.Time.now()))
                dest = ping_resp.cst.src
                src = ping_resp.cst.dest
                owtt = ping_resp.one_way_travel_time
                tat = ping_resp.tat
                measured_range = owtt * self.sound_speed
                # Convert the timestamp to a ROS Time object
                # NOTE: The timestamp is in float seconds, so we can convert it to a ROS Time object
                timestamp_ns = ping_resp.cst.toa - rospy.Duration.from_sec(owtt)
                # Convert the timestamp to seconds (rather than ns)
                timestamp_sec = rospy.Time.from_sec(timestamp_ns.to_sec())
                # Log all the fields
                rospy.loginfo("[%s] Ping Response: timestamp=%s, src=%d, dest=%d, owtt=%.4f, tat= %.4f, measured_range=%.4f" % (rospy.Time.now(), timestamp_sec, src, dest, owtt, tat, measured_range))
                # TODO: Verify this is rostime
                # Log the ping response
                src_chr = chr(ord("A") + src)
                if dest < self.num_agents:
                    dest_chr = chr(ord("A") + dest)
                else:
                    dest_chr = "L%d" % (dest - self.num_agents)
                self.range_data.append([timestamp_sec.to_sec(), src, dest, owtt, measured_range])
                #NOTE: This allows for preintegration between ranges to landmarks and ranges to agents
                self.request_preintegration(timestamp_ns, True) # Request a relative pose measurement
                if target_addr == first_tgt:
                    self.ping_slot_1_attempted = True
                elif target_addr == second_tgt:
                    self.ping_slot_2_attempted = True
                self.poke_ping_cycle(3)
                return
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
            rospy.loginfo(f"Attempting to preintegrate between {ti} and {tj}")
            response = self.preintegrate_imu(tj)

            if not response.success:
                rospy.logerr(f"Preintegration failed: {response.error_message}") # Generate a placeholder response for comms testing
                response = PreintegrateIMUResponse(
                    success=True,
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
                    # pose_delta is a PoseWithCovarianceStamped message
                    position = np.array(x_ij.pose.pose.position)
                    orientation = np.array(x_ij.pose.pose.orientation)
                    # Convert the covariance to a numpy array and reshape it to 6x6
                    covariance = np.array(x_ij.pose.covariance).reshape((6, 6))
                    sigmas = np.sqrt(np.diag(covariance))
                    # Store the pose ing the preintegration data
                    self.preintegration_data.append({
                        "ti": ti,
                        "tj": tj,
                        "key1": key1,
                        "key2": key2,
                        "position": position,
                        "orientation": orientation,
                        "sigmas": sigmas
                    })
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
        # Log the reciept (disabled for dockside)
        #rospy.loginfo("[%s] Received GPS data" % pose_time)
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
                self.message_on_safe = True  # Messages are not allowed when not in water
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
        # Set the initial prior factor message
        init_prior_msg = InitPrior()
        init_prior_msg.local_addr = int(self.local_address)
        init_prior_msg.full_index = int(500)
        # Create a maximal length version of the initial prior factor
        # Iniial position is a array of 3 int16
        init_prior_msg.initial_position = [32767, 32767, 32767]
        # Initial orientation is a array of 4 int16
        init_prior_msg.initial_orientation = [32767, 32767, 32767, 32767]
        # Initial sigmas is a array of 6 uint16
        init_prior_msg.initial_sigmas = [65535, 65535, 65535, 65535, 65535, 65535]
        # Pass the position to the pose time lookup
        init_symbol = local_chr + str(0)
        self.pose_time_lookup[init_symbol] = rospy.Time.now()
        # Stage the initial prior factor message for sending
        self.staged_init_prior = init_prior_msg
        return

    def build_partial_graph(self):
        """This function builds the partial graph from the cycle graph data
        - It checks that the graph is connected and notes any discrepancies (does not currently patch them, just fails)
        - It check that the graph will fit in the PartialGraph.msg
        - It builds the partial graph message from the cycle graph data
        Returns:
            PartialGraph: The built partial graph message
        """
        max_position = [32767, 32767, 32767]
        max_orientation = [127, 127, 127, 127]
        max_sigmas = [255, 255, 255, 255, 255, 255]  # Maximal length sigmas
        partial_graph = PartialGraph()
        partial_graph.local_addr = int(self.local_address)
        partial_graph.full_index = int(500)  # Set the full index to a maximal value
        partial_graph.num_poses = int(6)
        partial_graph.relative_pos_0 = max_position  # Maximal length position
        partial_graph.relative_rot_0 = max_orientation  # Maximal length orientation
        partial_graph.unique_sigmas_0 = max_sigmas  # Maximal length sigmas
        partial_graph.relative_pos_1 = max_position  # Maximal length position
        partial_graph.relative_rot_1 = max_orientation  # Maximal length orientation
        partial_graph.unique_sigmas_1 = max_sigmas  # Maximal length sigmas
        partial_graph.relative_pos_2 = max_position  # Maximal length position
        partial_graph.relative_rot_2 = max_orientation  # Maximal length orientation
        partial_graph.unique_sigmas_2 = max_sigmas  # Maximal length sigmas
        partial_graph.relative_pos_3 = max_position  # Maximal length position
        partial_graph.relative_rot_3 = max_orientation  # Maximal length orientation
        partial_graph.unique_sigmas_3 = max_sigmas  # Maximal length sigmas
        partial_graph.relative_pos_4 = max_position  # Maximal length position
        partial_graph.relative_rot_4 = max_orientation  # Maximal length orientation
        partial_graph.unique_sigmas_4 = max_sigmas  # Maximal length sigmas
        partial_graph.relative_pos_5 = max_position  # Maximal length position
        partial_graph.relative_rot_5 = max_orientation  # Maximal length orientation
        partial_graph.local_index_0 = int(5)
        partial_graph.remote_addr_0 = int(5)
        partial_graph.meas_range_0 = int(65535)  # Maximal length range
        partial_graph.local_index_1 = int(5)
        partial_graph.remote_addr_1 = int(5)
        partial_graph.meas_range_1 = int(65535)  # Maximal length range
        partial_graph.local_index_2 = int(5)
        partial_graph.remote_addr_2 = int(5)
        partial_graph.meas_range_2 = int(65535)  # Maximal length range
        partial_graph.local_index_3 = int(5)
        partial_graph.remote_addr_3 = int(5)
        partial_graph.meas_range_3 = int(65535)  # Maximal length range
        partial_graph.local_index_4 = int(5)
        partial_graph.remote_addr_4 = int(5)
        partial_graph.remote_index_4 = int(500)
        partial_graph.local_index_5 = int(5)
        partial_graph.remote_addr_5 = int(5)
        partial_graph.remote_index_5 = int(500)
        self.staged_partial_graph = partial_graph
        return partial_graph

    def summarize_range_data(self):
        """This function summarizes the range data collected upon node shutdown"""
        rospy.loginfo("[%s] Range Data Summary:" % rospy.Time.now())

        # Report the number of completed ranges to each dest
        range_summary = {}
        for entry in self.range_data:
            timestamp, src, dest, owtt, measured_range = entry
            if dest not in range_summary:
                range_summary[dest] = []
            range_summary[dest].append((timestamp, src, owtt, measured_range))
        for dest in range_summary.keys():
            # if there are no ranges to this dest, skip it
            if not range_summary[dest]:
                rospy.loginfo("[%s] No ranges to %s" % (rospy.Time.now(), chr(ord("A") + dest)))
            else:
                #Check if there is an entry that includes a measured_range
                has_measured_range = any(r[3] is not None for r in range_summary[dest])
                if not has_measured_range:
                    rospy.loginfo("[%s] Ranges to %s: %d / %d valid" % (rospy.Time.now(), chr(ord("A") + dest), 0, len(range_summary[dest])))
                else:
                    # Extract valid entries (measured_range not None)
                    valid_range_set = [r for r in range_summary[dest] if r[3] is not None]

                    # Extract only the ranges and timestamps
                    valid_ranges = [r[3] for r in valid_range_set]
                    valid_timestamps = [r[0] for r in valid_range_set]

                    # Compute statistics
                    num_valid = len(valid_ranges)
                    min_range = np.min(valid_ranges)
                    max_range = np.max(valid_ranges)
                    mean_range = np.mean(valid_ranges)
                    std_range = np.std(valid_ranges)

                    # Compute time delta (assumes timestamps are rospy.Time or float)
                    time_delta = valid_timestamps[-1] - valid_timestamps[0]

                    # If rospy.Time, convert to seconds
                    if hasattr(time_delta, 'to_sec'):
                        time_delta_sec = time_delta.to_sec()
                    else:
                        time_delta_sec = time_delta  # already float

                    # Log or return as needed
                    print(f"Valid Ranges: {num_valid}")
                    print(f"Min: {min_range}, Max: {max_range}, Mean: {mean_range}, Std Dev: {std_range}")
                    print(f"Time Span: {time_delta_sec} seconds")
                    rospy.loginfo("[%s] Ranges to %s: %d / %d valid, From %.2f - %.2fm, Mean: %.2fm, dT: %.2fsec" % (rospy.Time.now(), chr(ord("A") + dest), num_valid, len(range_summary[dest]), min_range, max_range, mean_range, time_delta_sec))
        return

    def log_ranges_to_csv(self):
        """ Log ranges, cst and xst data to csv"""
        if not self.xst_data:
            rospy.logwarn("[%s] No Signals Transmitted" % rospy.Time.now())
            return
        else:
            xst_timestamp = self.xst_data[0][0]

        # Create the csv files:
        log_dir = "/ros/logs/"
        #log_dir = "/home/morrisjp/bags/June"
        range_file = join(log_dir, f"range_data_{xst_timestamp}.csv")
        cst_file = join(log_dir, f"cst_data_{xst_timestamp}.csv")
        xst_file = join(log_dir, f"xst_data_{xst_timestamp}.csv")
        with open(range_file, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(["timestamp", "src", "dest", "owtt", "measured_range"])
            # Write data rows
            for entry in self.range_data:
                writer.writerow(entry)
            rospy.loginfo("[%s] Range Data Written to File at: %s" % (rospy.Time.now(),range_file))
        with open(cst_file, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(["timestamp", "src", "dest", "msg_type", "nframes", "snr_rss", "snr_in", "snr_out", "dop(m/s)", "stddev_noise"])
            # Write data rows
            for entry in self.cst_data:
                writer.writerow(entry)
            rospy.loginfo("[%s] CACST Data Written to File at: %s" % (rospy.Time.now(),cst_file))
        with open(xst_file, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(["timestamp", "src", "dest", "msg_type", "nframes", "nbytes"])
            # Write data rows
            for entry in self.xst_data:
                writer.writerow(entry)
            rospy.loginfo("[%s] CAXST Data Written to File at: %s" % (rospy.Time.now(),xst_file))
        return

    def log_pim_to_csv(self):
        """Log the preintegration data to a csv file"""
        if not self.preintegration_data:
            rospy.logwarn("[%s] No Preintegration Data" % rospy.Time.now())
            return
        else:
            preintegration_file = join("/ros/logs/", f"preintegration_data_{rospy.Time.now()}.csv")
            with open(preintegration_file, mode='w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write header
                writer.writerow(["ti","tj","key1", "key2", "position_x", "position_y", "position_z", "q_x", "q_y", "q_z", "q_w", "sig_x", "sig_y", "sig_z, sig_r, sig_p, sig_y"])
                # Write data rows
                for entry in self.preintegration_data:
                    position = entry["position"]
                    orientation = entry["orientation"]
                    sigmas = entry["sigmas"]
                    writer.writerow([
                        entry["ti"].to_sec(),
                        entry["tj"].to_sec(),
                        entry["key1"],
                        entry["key2"],
                        position[0], position[1], position[2],
                        orientation[0], orientation[1], orientation[2], orientation[3],
                        sigmas[0], sigmas[1], sigmas[2], sigmas[3], sigmas[4], sigmas[5]
                    ])
            rospy.loginfo("[%s] Preintegration Data Written to File at: %s" % (rospy.Time.now(), preintegration_file))
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
        # Summarize the range data collected
        cycle_mgr.log_ranges_to_csv()
        cycle_mgr.log_pim_to_csv()
        cycle_mgr.summarize_range_data()
        rospy.loginfo("[%s] Comms Cycle Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Comms Cycle Mgr Exiting")
        exit(0)