#!/usr/bin/env python3
import rospy
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
        self.num_landmarks = int(rospy.get_param("num_landmarks", 2))
        self.landmarks = rospy.get_param("landmarks", {"L0":[-71.7845,-39.6078,1.5], "L1":[65.0832,25.6598,1.5]}) # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = float(rospy.get_param("sound_speed", 1500))
        # Variables for addressing
        self.modem_addresses = {}
        self.cycle_target_mapping = {}
        # Variables for tdma and cycle execution
        self.tdma_status = TdmaStatus()
        self.tdma_cycle_sequence = 0
        self.queue_status = QueueStatus()
        self.msg_preload = 5 # seconds to allow for encoding the message before the tdma window opens
        # Variables for acomms event topic
        self.ping_method = "ping with payload"
        self.ping_timeout = 3 # seconds
        self.message_on_safe = False # True if messages not allowed
        self.ping_on_safe = True # True if pings not allowed
        self.ping_slot_1_attempted = False # True if the first ping slot has been attempted
        self.ping_slot_2_attempted = False # True if the second ping slot has been attempted
        # Variables for external sensors
        self.gps_fix = [[1,2,3],[0,0,0,1],[1.7,1.7,3.5,0.1,0.1,0.1]] # [position, orientation, covariance]
        # Variables for message handling
        self.staged_init_prior = InitPrior()
        self.staged_partial_graph = PartialGraph()
        self.range_data = []
        self.partial_graph_data = {}
        self.cycle_graph_data = {}
        self.inbound_init_priors = {}
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
        self.preintegrate_imu = rospy.ServiceProxy("preintegrate_imu", PreintegrateIMU)
        rospy.loginfo("[%s] Services ready, initializing topics" % rospy.Time.now())
        # Initialize topics
        self.tdma_from_modem = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_status, queue_size=1)
        self.queue_status_from_modem = rospy.Subscriber("modem/queue_status", QueueStatus, self.on_queue_status, queue_size=1)
        rospy.loginfo("[%s] TDMA Status msg registered Configured" % rospy.Time.now())
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
            #NOTE:
            # Smoothing results in a relative pose connecting A0-A5, but message format only allows
            # for relative poses with sequential indices to be published.
            # So we can't sequence a partial graph covering A0-A6, A6-A7, A7-A8, A8-A9, A9-A10
            # So instead we transmit A5-A6, A6-A7, A7-A8, A8-A9, A9-A10, where the measurements under A5-A6
            # actually represent A0-A6.
            # To account for this, the factor_graph_mgr (synchronize_partial_graphs) checks if the partial graph
            # is continguous with the previous graph, and if not, will re-key the graph to be contiguous.
            # NOTE: we do not roll back the pose_time_lookup, so its possible that the smoothing will result in
            # different sequence numbers for the two.
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

        # If we need to send messages to other modems:
        if self.num_agents >= 1:
        # Check if we are in the message preload phase:
        #NOTE: This normally happens when the vehicle is not active
            if (we_are_active and remaining_sec_in_slot < self.msg_preload) or (we_are_active==False and time_to_next_active < self.msg_preload) and self.queue_status.total_message_count == 0 and self.message_on_safe == False:
                #rospy.loginfo("[%s] Preload Status: Remaining Seconds: %.2f, Active: %s, In Queue %d" % (rospy.Time.now(), remaining_sec_in_slot, we_are_active, self.queue_status.total_message_count))
                # Check if we are in the initial prior phase:
                if self.tdma_cycle_sequence == 0:
                    # Check if init prior is staged, if not, build it:
                    if self.staged_init_prior is not None and self.staged_init_prior.initial_position[0] == 0:
                        self.build_init_prior()
                        self.init_prior_pub.publish(self.staged_init_prior)
                        # Bypass the modem and publish directly to the graph manager
                        self.init_prior_bypass_pub.publish(self.staged_init_prior)
                        #self.loaded_msg = True
                        rospy.loginfo("[%s] Published Init Prior %s" % (rospy.Time.now(), self.staged_init_prior))
                        #self.tdma_cycle_sequence += 1
                    else:
                        pass
                # If we are in the partial graph phase:
                elif self.queue_status.total_message_count == 0:
                    # If we don't have a valid partial graph staged, send a template
                    if self.staged_partial_graph.num_poses == 0:
                        #rospy.loginfo("[%s] No partial graph data to send" % rospy.Time.now())
                        self.partial_graph_pub.publish(PartialGraph())
                    else:
                        rospy.loginfo("[%s] Publishing Partial Graph %s" % (rospy.Time.now(),msg_id))
                        #rospy.loginfo("[%s] Staged Partial Graph %s" % (rospy.Time.now(),self.staged_partial_graph))
                        # Publish to the modem
                        self.partial_graph_pub.publish(self.staged_partial_graph)
                        # Bypass the modem and publish directly to the graph manager
                        self.partial_graph_pub.publish(self.staged_partial_graph)
                else:
                    pass # We've already loaded
                self.message_on_safe = True  # If we are in the preload phase, we should not ping
        else:
            # This is an escape to prevent the ping cycle from executing if we have no other agents or landmarks
            rospy.logerr("[%s] No other agents or landmarks to ping" % rospy.Time.now())

        # If we are active, we need to execute the ping cycle
        if we_are_active == True:
            #rospy.loginfo("[%s] Active Status: Slot %s, Remaining Seconds: %.2f, Active: %s, Ping Safety: %s" % (rospy.Time.now(), current_slot, remaining_sec_in_slot, we_are_active, self.ping_on_safe))
            # If we have just entered the active slot:
            if self.tdma_status.we_are_active == False:
                rospy.loginfo("[%s] TDMA Active Slot Started, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
                self.message_on_safe = False  # If we are in the active slot, we should allow messages
                self.ping_slot_1_attempted = False
                self.ping_slot_2_attempted = False
            elif self.ping_on_safe == False:
                #rospy.loginfo("[%s] Attempting to ping- P1 Status: %s, P2 Status: %s" % (rospy.Time.now(), self.ping_slot_1_attempted, self.ping_slot_2_attempted))
                first_tgt, second_tgt = self.cycle_target_mapping[current_slot]
                if self.ping_slot_1_attempted == False and self.ping_slot_2_attempted == False:
                    # Execute the ping cycle for the current slot
                    self.ping_slot_1_attempted = True
                    self.send_ping(first_tgt)
                    rospy.sleep(self.ping_timeout)
                elif self.ping_slot_1_attempted == True and self.ping_slot_2_attempted == False:
                    self.ping_slot_2_attempted = True
                    self.send_ping(second_tgt)
                    rospy.sleep(self.ping_timeout)
                    rospy.loginfo("[%s] Ping Cycle Completed, Time to Complete: %.2f seconds" % (rospy.Time.now(), elapsed_time_in_slot))
                else:
                    # Log that the ping process is completed and note how long it took to get there
                    pass
            else:
                # If we are not allowed to ping, log that we are waiting for the next cycle
                #rospy.loginfo("[%s] Pings not allowed, waiting for next cycle" % rospy.Time.now())
                pass
        else:
            # If we are not active, do nothing
            #rospy.loginfo("[%s] Not in active slot, waiting for next cycle" % rospy.Time.now())
            pass

        # Regardless, update TDMA
        self.tdma_status = msg
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
            self.tdma_cycle_sequence += 1
        elif self.staged_init_prior == InitPrior():
            # If we are not initialized, we need to build the init prior
            self.staged_init_prior = self.build_init_prior()
            self.tdma_cycle_sequence = 0
        else:
            # We are initialized, but others are not, so we need to reset the cycle
            self.partial_graph_data.clear()
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
        if msg_count > 0 or self.tdma_status.we_are_active == False:
            self.ping_on_safe = True  # If there are messages in the queue, we should not ping
            #rospy.loginfo("[%s] Queue Status: %d messages in queue" % (rospy.Time.now(), msg_count))
            if msg_count > 1:
                rospy.logwarn("[%s] Queue Status: %d messages in queue" % (rospy.Time.now(), msg_count))
        if self.queue_status.total_message_count > 0 and msg_count == 0:
            # We have flushed the queue and should now allow ping
            rospy.sleep(3)
            self.ping_on_safe = False
            self.ping_slot_1_attempted = False
            self.ping_slot_2_attempted = False
            time_to_clear_queue = self.tdma_status.slot_duration_seconds - self.tdma_status.remaining_slot_seconds
            # Report the time to clear the queue to 2 decimal places
            rospy.loginfo("[%s] Queue Cleared, Time to Clear: %.2f seconds" % (rospy.Time.now(), time_to_clear_queue))
        elif self.tdma_status.we_are_active == False and msg_count == 0:
            # If we are not active and there are messages in the queue, we should not ping
            self.ping_on_safe = False
        else:
            pass
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
            if data[1] == "PNG" and dest == self.local_address:
                self.request_preintegration(recieved_ping_time, True) # Request a relative pose measurement
                rospy.loginfo("[%s] Received Ping from %s" % (recieved_ping_time, chr(ord("A") + src)))
            elif data[1] == "PNG":
                rospy.loginfo("[%s] Overheard Ping from %s to %s" % (recieved_ping_time, chr(ord("A") + src), chr(ord("A") + dest)))
            else:
                rospy.logerr("[%s] Received $CACMA with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMR": # Modem-to-host acknowledgement of a ping response
            src, dest, recieved_ping_time = parse_nmea_cacmr(data)
            if data[0] == "PNR" and src == self.local_address:
                rospy.loginfo("[%s] Received Ping Response from %s" % (recieved_ping_time, chr(ord("A") + dest)))
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
                self.range_data.append([rospy.Time.now().to_sec() -self.ping_timeout/2, self.local_address, target_addr, None, None])
            else:
                rospy.loginfo("[%s] Ping Successful: "% (rospy.Time.now()))
                dest = ping_resp.cst.src
                src = ping_resp.cst.dest
                owtt = ping_resp.one_way_travel_time
                tat = ping_resp.tat
                measured_range = owtt * self.sound_speed
                # Convert the timestamp to a ROS Time object
                # NOTE: The timestamp is in float seconds, so we can convert it to a ROS Time object
                timestamp = ping_resp.cst.toa - rospy.Duration.from_sec(owtt)

                # Log all the fields
                rospy.loginfo("[%s] Ping Response: timestamp=%s, src=%d, dest=%d, owtt=%.4f, tat= %.4f, measured_range=%.4f" % (rospy.Time.now(), timestamp, src, dest, owtt, tat, measured_range))
                # TODO: Verify this is rostime
                # Log the ping response
                src_chr = chr(ord("A") + src)
                if dest < self.num_agents:
                    dest_chr = chr(ord("A") + dest)
                else:
                    dest_chr = "L%d" % (dest - self.num_agents)
                self.range_data.append([timestamp, src, dest, owtt, measured_range])
                #NOTE: This allows for preintegration between ranges to landmarks and ranges to agents
                self.request_preintegration(timestamp, True) # Request a relative pose measurement
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
            #response = self.preintegrate_imu(ti, tj)

            # Generate a placeholder response for comms testing
            response = PreintegrateIMUResponse(
                success=True,
                pose_delta=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    covariance=[1.0] * 36  # Placeholder covariance
                    )
                )
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
                    pose = x_ij.pose
                    position = np.array(pose.position)
                    orientation = np.array(pose.orientation)
                    covariance = np.array(response.pose_delta.covariance).reshape((6, 6))
                    sigmas = np.sqrt(np.diag(covariance))
                    # Add to the cycle graph data
                    graph_id = "BTWN_%s_%s" % (key1, key2)
                    self.partial_graph_data[graph_id] = {
                        "key1": key1,
                        "key2": key2,
                        "position": np.array(position),
                        "orientation": np.array(orientation),
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
        initial_position, initial_orientation, initial_sigmas = encode_init_prior_data_as_int(self.gps_fix[0],self.gps_fix[1],self.gps_fix[2])
        # Set the initial prior factor message
        init_prior_msg = InitPrior()
        init_prior_msg.local_addr = int(self.local_address)
        init_prior_msg.full_index = int(self.modem_addresses[local_chr][1])
        init_prior_msg.initial_position = initial_position
        init_prior_msg.initial_orientation = initial_orientation
        init_prior_msg.initial_sigmas = initial_sigmas
        # Pass the position to the pose time lookup
        init_symbol = local_chr + str(0)
        self.pose_time_lookup[init_symbol] = rospy.Time.now()
        # Stage the initial prior factor message for sending
        self.staged_init_prior = init_prior_msg
        # Publish the initial prior factor to the modem
       #self.init_prior_pub.publish(self.staged_init_prior)
        # Bypass the modem and publish directly to the graph manager
        #self.init_prior_bypass_pub.publish(self.staged_init_prior)
        self.inbound_init_priors[local_chr] = {
            "key": local_symbol,
            "position": initial_position,
            "orientation": initial_orientation,
            "sigmas": initial_sigmas
        }
        # Log that we've sent the message
        #rospy.loginfo("[%s] Created Initial Prior Factor: %s" % (rospy.Time.now(), self.staged_init_prior)
        #TODO: This correctly sends the message, but bypasses the staging feature (and redundant message)
        # self.init_prior_pub.publish(self.staged_init_prior)
        # # Bypass the modem and publish directly to the graph manager
        # self.init_prior_bypass_pub.publish(self.staged_init_prior)
        # #self.loaded_msg = True
        # rospy.loginfo("[%s] Published Init Prior %s" % (rospy.Time.now(), self.staged_init_prior))
        # #self.tdma_cycle_sequence += 1
        # self.staged_init_prior = InitPrior()  # Clear the staged init prior after publishing
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
        if self.num_agents == 1:
            #rospy.loginfo("[%s] No partial graph data to send" % rospy.Time.now())
            return partial_graph
        else:
            # Check that the graph is the right size, is feasible and is connected:
            graph_check = check_partial_graph_msg(self.partial_graph_data)
            if not graph_check:
                rospy.logerr("[%s] Partial graph data check failed" % rospy.Time.now())
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
                    ranges = len([r for r in range_summary[dest] if r[3] is not None])
                    mean = np.mean([r[3] for r in range_summary[dest] if r[3] is not None])
                    sigma = np.std([r[3] for r in range_summary[dest] if r[3] is not None])
                    rospy.loginfo("[%s] Ranges to %s: %d / %d valid, Mean: %.2f, Sigma: %.2f" % (rospy.Time.now(), chr(ord("A") + dest), ranges, len(range_summary[dest]), mean, sigma))

        # Print the range data to a log file:
        if not self.range_data:
            rospy.loginfo("[%s] No range data collected" % rospy.Time.now())
            return
        first_timestamp = self.range_data[0][0]
        log_file = join("/home/morrisjp/bags/June", f"range_data_{first_timestamp}.txt")
        with open(log_file, "w") as f:
            for entry in self.range_data:
                timestamp, src, dest, owtt, measured_range = entry
                f.write(f"{timestamp}, {src}, {dest}, {owtt}, {measured_range}")
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
        cycle_mgr.summarize_range_data()
        rospy.loginfo("[%s] Comms Cycle Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Comms Cycle Mgr Exiting")
        exit(0)