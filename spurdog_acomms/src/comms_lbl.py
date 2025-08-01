#!/usr/bin/env python3
import rospy
import csv
from os.path import dirname, join, abspath
import numpy as np
# from datetime import datetime
# import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32, Bool
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseWithCovarianceStamped
from ros_acomms_msgs.msg import(
    TdmaStatus, QueueStatus, PingReply, CST, XST
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)
from spurdog_acomms.msg import(
    Bar30SoundSpeed, RangeFactorStamped, PoseFactorStamped, AcommsCycleStatus, ReceivedSignalStats
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

class CycleManager:
    """This is a node to run the comms cycle for the vehicle.
        - This is a lightweight version designed to obtain the maximum amount of range data
        - It is designed to run on a single agent with an arbitary number of landmarks
        - This is configured to ping one target, get a response and immediately ping the next target
    """
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        # Get current namespace, e.g. "/actor_0/comms_lbl"
        full_ns = rospy.get_namespace()           # e.g., "/actor_0/comms_lbl/"
        rospy.loginfo("[%s] Full Namespace as %s"%(rospy.Time.now(), full_ns))
        # Config:
        self.local_address = int(rospy.get_param(full_ns + "modem_address", 0))
        self.num_agents = int(rospy.get_param(full_ns + "num_agents", 1))
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
        self.ping_timeout = float(rospy.get_param("~ping_timeout", 5))
        self.sound_speed = float(rospy.get_param("~sound_speed", 1486))
        self.range_sigma = float(rospy.get_param("~sigma_range", 1)) # meters, this is the expected error in the range measurement
        # Variables for Logging:
        self.range_data = []
        self.preintegration_data = []
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
        self.send_ping(self.modem_addresses["L0"][0])  # Send a ping to the first landmark

    def configure_comms_cycle(self):
        """This function configures the comms cycle for the vehicle.
        """
        # Get the modem addresses and cycle targets
        self.modem_addresses = configure_modem_addresses(self.num_agents, self.num_landmarks, self.local_address)
        self.address_to_name = {v[0]: k for k, v in self.modem_addresses.items()}
        rospy.loginfo("[%s] Address to name: %s" % (rospy.Time.now(), self.address_to_name))
        # make a list of modem addresses that don't include the local address (the int address, not the name)
        ping_addresses = [value[0] for value in self.modem_addresses.values() if value[0] != self.local_address]
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
            if data[1] == "PNG" and dest == self.local_address:
                self.request_preintegration(rcvd_stamp, True) # Request a relative pose measurement
                self.acomms_event_pub.publish("priority=2,pattern=([0.0.0.255]:0.5)([100.0.150.50]:1.0),cycles=1")
                rospy.loginfo("[%s] Received Ping from %s" % (recieved_ping_time, chr(ord("A") + self.address_to_name[src])))
            elif data[1] == "PNG":
                rospy.loginfo("[%s] Overheard Ping from %s to %s" % (recieved_ping_time, self.address_to_name[src], self.address_to_name[dest]))
            else:
                rospy.logerr("[%s] Received $CACMA with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMR": # Modem-to-host acknowledgement of a ping response
            src, dest, recieved_ping_time, owtt = parse_nmea_cacmr(data)
            if data[0] == "PNR" and src == self.local_address:
                self.acomms_event_pub.publish("priority=2,pattern=([0.0.0.255]:0.5)([0.255.0.50]:1.0),cycles=1")
                rospy.loginfo("[%s] Received Ping Response from %s" % (recieved_ping_time, self.address_to_name[dest]))
            elif data[0] == "PNR":
                rospy.loginfo("[%s] Overheard Ping Response from %s to %s" % (recieved_ping_time, self.address_to_name[src], self.address_to_name[dest]))
            else:
                rospy.logerr("[%s] Received $CACMR with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CARFP" and data[5] == "-1": # Modem-to-host acknowledgement of a minipacket ping payload
            src, dest, recieved_msg_time, num_frames, payload = parse_nmea_carfp(data)
            if recieved_msg_time == None or not src == None or not dest == None:
                rospy.logerr("[%s] CARFP message is missing required fields" % rospy.Time.now())
                return
            elif dest == self.local_address:
                rospy.loginfo("[%s] Received Ping from %s with payload %s" % (recieved_msg_time, self.address_to_name[src], payload))
            elif dest != self.local_address:
                rospy.logerr("[%s] Overheard Ping-related $CARFP from %s to %s with paylaod %s" % (recieved_msg_time, self.address_to_name[src], self.address_to_name[dest], payload))
            else:
                rospy.logerr("[%s] Received $CARFP with unexpected data: %s" % (rospy.Time.now(), data))

        # elif nmea_type == "$CACST": # Modem-to-host report of signal recieved
        #     cst_statistics = parse_nmea_cacst(data)
        #     self.cst_data.append(cst_statistics)

        # elif nmea_type == "$CAXST": # Modem-to-host report of signal transmitted
        #     xst_statistics = parse_nmea_caxst(data)
        #     self.xst_data.append(xst_statistics)

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

    def send_ping(self, target_addr):
        """This function sends a ping to the modem
        Args:
            target_addr (int): the target address
            symbol (str): the local key "A1" to put in the payload
        """
        first_tgt, second_tgt = self.cycle_target_mapping["0"]
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
                if target_addr == first_tgt:
                    self.send_ping(second_tgt)  # Attempt the second target
                elif target_addr == second_tgt:
                    self.send_ping(first_tgt)  # Attempt the first target
                self.range_data.append([None, rospy.Time.now().to_sec()-self.ping_timeout/2, rospy.Time.now().to_sec(), self.local_address, target_addr, None, None, None, None, None, None])
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
                # Send the range factor message
                range_factor_msg = RangeFactorStamped()
                range_factor_msg.header.stamp = timestamp_sec
                range_factor_msg.header.frame_id = "modem"
                range_factor_msg.key1 = chr(ord("A") + self.local_address) + str(self.modem_addresses[chr(ord("A") + self.local_address)][1])
                range_factor_msg.key2 = self.address_to_name[target_addr]
                range_factor_msg.meas_range = measured_range
                range_factor_msg.range_sigma = self.range_sigma
                range_factor_msg.depth1 = self.depth
                range_factor_msg.depth2 = -1.83
                self.range_factor_pub.publish(range_factor_msg)
                # Log all the fields
                rospy.loginfo("[%s] Ping Complete %s to %s: timestamp=%s, owtt=%.4f, tat= %.4f, measured_range=%.2f" % (
                    rospy.Time.now(), self.address_to_name[src], self.address_to_name[dest], timestamp_sec, owtt, tat, measured_range))
                # self.range_data.append([timestamp_sec.to_sec(), src, dest, owtt, measured_range])
                # Request preintegration
                self.request_preintegration(timestamp_ns, True) # Request a relative pose measurement (and advance the pose index)
                # Publish a pose factor message
                # Log the pose factor
                # pose_factor_msg = PoseFactorStamped()
                # pose_factor_msg.header.stamp = timestamp_ns
                # pose_factor_msg.header.frame_id = "modem"
                # self.pose_factor_pub.publish(pose_factor_msg)
                # increment the pose index
                local_chr = chr(ord("A") + self.local_address)
                self.modem_addresses[local_chr][1] += 1
                self.pose_time_lookup[local_chr + str(self.modem_addresses[local_chr][1])] = timestamp_sec
                if target_addr == first_tgt:
                    self.send_ping(second_tgt)  # Attempt the second target
                elif target_addr == second_tgt:
                    self.send_ping(first_tgt)  # Attempt the first target
                return
        except rospy.ServiceException as e:
            rospy.logerr("[%s] Ping Service Call Failed: %s" % (rospy.Time.now(), e))
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
            response = self.preintegrate_imu(rospy.Time(0), tj)
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
                # Store the pose ing the preintegration data
                # self.preintegration_data.append({
                #     "ti": ti,
                #     "tj": tj,
                #     "key1": key1,
                #     "key2": key2,
                #     "position": position,
                #     "orientation": orientation,
                #     "sigmas": sigmas
                # })
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
            #self.request_preintegration(rospy.Time.now(), adv_pose=False)
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
        # Append to range data
        #self.range_data.append([xst_time, None, None, src, dest, None, None, None, None, None, None])
        return

    def on_range_log(self, msg: PingReply):
        """This function logs the range data from the modem
        Args:
            msg (PingReply): The range data from the modem
        """
        # Get timestamp from header
        message_timestamp = msg.header.stamp.to_sec()
        range_timestamp = rospy.Time(msg.cst.toa.secs, msg.cst.toa.nsecs).to_sec(),
        src = msg.dest #NOTE: inverted due to ping reply
        dest = msg.src
        owtt = msg.owtt
        measured_range = owtt * self.sound_speed if owtt is not None else None
        #tat = msg.tat
        #snr_in = msg.snr_in
        #snr_out = msg.snr_out
        # tx_level = msg.tx_level
        # mfd_peak = msg.cst.mfd_peak
        # mfd_pow = msg.cst.mfd_pow
        # mfd_ratio = msg.cst.mfd_ratio
        # mfd_spl = msg.cst.mfd_spl
        # agn = msg.cst.agn
        # shift_ainp = msg.cst.shift_ainp
        # shift_ain = msg.cst.shift_ain
        # shift_aout = msg.cst.shift_aout
        # shift_mfd = msg.cst.shift_mfd
        # shift_p2b = msg.cst.shift_p2b
        # rate = msg.cst.rate_num
        # psk_error = msg.cst.psk_error
        # packet_type = msg.cst.packet_type
        # num_frames = msg.cst.num_frames
        # bad_frames = msg.cst.bad_frames_num
        # snr_rss = msg.cst.snr_rss
        #stddev_noise = msg.cst.noise
        # mse_error = msg.cst.mse
        # dqf = msg.cst.dqf
        #dop = msg.cst.dop
        # Update the cycle status
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
        # Add it to the existing XST-based data
        # for entry in self.range_data:
        #     # If the entry has been filled, ignore
        #     if entry[1] is not None:
        #         continue
        #     # If the entry matches the src and dest, fill it
        #     elif entry[3] == src and entry[4] == dest:
        #         entry[1] = range_timestamp
        #         entry[2] = message_timestamp
        #         entry[5] = owtt
        #         entry[6] = measured_range
        #         entry[7] = dop
        #         entry[8] = stddev_noise
        #         entry[9] = snr_in
        #         entry[10] = snr_out
        #     else:
        #         # Make a new entry if no match is found
        #         self.range_data.append([
        #             None,  # xst_timestamp
        #             range_timestamp,  # range_timestamp
        #             message_timestamp,  # cst_timestamp
        #             src,  # src
        #             dest,  # dest
        #             owtt,  # owtt
        #             measured_range,  # measured_range
        #             dop,  # dop(m/s)
        #             stddev_noise,  # stddev_noise
        #             snr_in,  # snr_in
        #             snr_out  # snr_out
        #         ])
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

    # def summarize_range_data(self):
    #     """This function summarizes the range data collected upon node shutdown"""
    #     rospy.loginfo("[%s] Range Data Summary:" % rospy.Time.now())

    #     # Report the number of completed ranges to each dest
    #     range_summary = {}
    #     for entry in self.range_data:
    #         xst_timestamp, range_timestamp, cst_timestamp, src, dest, owtt, measured_range, dop, stddev_noise, snr_in, snr_out = entry
    #         if dest not in range_summary:
    #             range_summary[dest] = []
    #         range_summary[dest].append(
    #             (xst_timestamp, range_timestamp, cst_timestamp, measured_range, owtt, dop, stddev_noise, snr_in, snr_out)
    #         )
    #     for dest in range_summary.keys():
    #         # if there are no ranges to this dest, skip it
    #         if not range_summary[dest]:
    #             rospy.loginfo("[%s] No ranges to %s" % (rospy.Time.now(), chr(ord("A") + dest)))
    #         else:
    #             #Check if there is an entry that includes a measured_range
    #             has_measured_range = any(r[3] is not None for r in range_summary[dest])
    #             if not has_measured_range:
    #                 rospy.loginfo("[%s] Ranges to %s: 0 / %d valid" % (rospy.Time.now(), chr(ord("A") + dest), len(range_summary[dest])))
    #             else:
    #                 # Extract valid entries (measured_range not None)
    #                 valid_range_set = [r for r in range_summary[dest] if r[3] is not None]

    #                 # Extract only the ranges and timestamps
    #                 valid_ranges = [r[3] for r in valid_range_set]
    #                 valid_timestamps = [r[1] for r in valid_range_set]

    #                 # Compute statistics
    #                 num_valid = len(valid_ranges)
    #                 min_range = np.min(valid_ranges)
    #                 max_range = np.max(valid_ranges)
    #                 mean_range = np.mean(valid_ranges)
    #                 std_range = np.std(valid_ranges)

    #                 # Compute time delta (assumes timestamps are rospy.Time
    #                 time_delta = rospy.Time.from_sec(max(valid_timestamps)) - rospy.Time.from_sec(min(valid_timestamps))

    #                 # Log or return as needed
    #                 print(f"Valid Ranges: {num_valid}")
    #                 print(f"Min: {min_range}, Max: {max_range}, Mean: {mean_range}, Std Dev: {std_range}")
    #                 print(f"Time Span: {time_delta} seconds")
    #                 rospy.loginfo("[%s] Ranges to %s: %d / %d valid, From %.2f - %.2fm, Mean: %.2fm, dT: %.2fsec" % (rospy.Time.now(), chr(ord("A") + dest), num_valid, len(range_summary[dest]), min_range, max_range, mean_range, time_delta))
    #     # # Report the average time, min, max and std dev of the difference between cell[2] - cell[0]
    #     # if not self.range_data:
    #     #     rospy.logwarn("[%s] No Range Data Collected" % rospy.Time.now())
    #     #     return
    #     # # Calculate the time differences
    #     # time_diffs_xst_cst = []
    #     # time_diffs_ri_rj = []
    #     # for i in range(len(self.range_data)):
    #     #     xst_timestamp, range_timestamp, cst_timestamp, src, dest, owtt, measured_range, dop, stddev_noise, snr_in, snr_out = self.range_data[i]
    #     #     if xst_timestamp is not None and cst_timestamp is not None:
    #     #         time_diffs_xst_cst.append(cst_timestamp - xst_timestamp)
    #     #     if range_timestamp[i] is not None and range_timestamp[i+1] is not None:
    #     #         time_diffs_ri_rj.append(range_timestamp[i+1] - range_timestamp[i])
    #     # # Calculate statistics
    #     # if time_diffs_xst_cst:
    #     #     avg_time_xst_cst = np.mean(time_diffs_xst_cst)
    #     #     min_time_xst_cst = np.min(time_diffs_xst_cst)
    #     #     max_time_xst_cst = np.max(time_diffs_xst_cst)
    #     #     std_time_xst_cst = np.std(time_diffs_xst_cst)
    #     #     rospy.loginfo("[%s] Average Time XST to CST: %.2f sec, Min: %.2f sec, Max: %.2f sec, Std Dev: %.2f sec" % (
    #     #         rospy.Time.now(), avg_time_xst_cst.to_sec(), min_time_xst_cst.to_sec(), max_time_xst_cst.to_sec(), std_time_xst_cst.to_sec()))
    #     # else:
    #     #     rospy.logwarn("[%s] No valid time differences between XST and CST" % rospy.Time.now())
    #     # if time_diffs_ri_rj:
    #     #     avg_time_ri_rj = np.mean(time_diffs_ri_rj)
    #     #     min_time_ri_rj = np.min(time_diffs_ri_rj)
    #     #     max_time_ri_rj = np.max(time_diffs_ri_rj)
    #     #     std_time_ri_rj = np.std(time_diffs_ri_rj)
    #     #     rospy.loginfo("[%s] Average Time between Ranges: %.2f sec, Min: %.2f sec, Max: %.2f sec, Std Dev: %.2f sec" % (
    #     #         rospy.Time.now(), avg_time_ri_rj.to_sec(), min_time_ri_rj.to_sec(), max_time_ri_rj.to_sec(), std_time_ri_rj.to_sec()))
    #     return

    # def log_ranges_to_csv(self):
    #     """ Log the range data to a csv file"""
    #     if not self.range_data:
    #         rospy.logwarn("[%s] No Range Data" % rospy.Time.now())
    #         return
    #     else:
    #         range_timestamp = self.range_data[0][0]

    #     # Create the csv file:
    #     log_dir = "/ros/logs/"
    #     #log_dir = "/home/morrisjp/bags/June"
    #     range_file = join(log_dir, f"range_data_{range_timestamp}.csv")
    #     with open(range_file, mode='w', newline='') as csvfile:
    #         writer = csv.writer(csvfile)
    #         # Write header
    #         writer.writerow(["xst_timestamp","range_timestamp", "cst_timestamp", "src", "dest", "owtt", "measured_range", "dop(m/s)", "stddev_noise", "snr_in", "snr_out"])
    #         # Write data rows
    #         for entry in self.range_data:
    #             writer.writerow(entry)
    #         rospy.loginfo("[%s] Range Data Written to File at: %s" % (rospy.Time.now(), range_file))
    #     return

    # def legacy_log_ranges_to_csv(self):
    #     """ Log ranges, cst and xst data to csv"""
    #     if not self.xst_data:
    #         rospy.logwarn("[%s] No Signals Transmitted" % rospy.Time.now())
    #         return
    #     else:
    #         xst_timestamp = self.xst_data[0][0]

    #     # Create the csv files:
    #     log_dir = "/ros/logs/"
    #     #log_dir = "/home/morrisjp/bags/June"
    #     range_file = join(log_dir, f"range_data_{xst_timestamp}.csv")
    #     cst_file = join(log_dir, f"cst_data_{xst_timestamp}.csv")
    #     xst_file = join(log_dir, f"xst_data_{xst_timestamp}.csv")
    #     with open(range_file, mode='w', newline='') as csvfile:
    #         writer = csv.writer(csvfile)
    #         # Write header
    #         writer.writerow(["timestamp", "src", "dest", "owtt", "measured_range"])
    #         # Write data rows
    #         for entry in self.range_data:
    #             writer.writerow(entry)
    #         rospy.loginfo("[%s] Range Data Written to File at: %s" % (rospy.Time.now(),range_file))
    #     with open(cst_file, mode='w', newline='') as csvfile:
    #         writer = csv.writer(csvfile)
    #         # Write header
    #         writer.writerow(["timestamp", "src", "dest", "msg_type", "nframes", "snr_rss", "snr_in", "snr_out", "dop(m/s)", "stddev_noise"])
    #         # Write data rows
    #         for entry in self.cst_data:
    #             writer.writerow(entry)
    #         rospy.loginfo("[%s] CACST Data Written to File at: %s" % (rospy.Time.now(),cst_file))
    #     with open(xst_file, mode='w', newline='') as csvfile:
    #         writer = csv.writer(csvfile)
    #         # Write header
    #         writer.writerow(["timestamp", "src", "dest", "msg_type", "nframes", "nbytes"])
    #         # Write data rows
    #         for entry in self.xst_data:
    #             writer.writerow(entry)
    #         rospy.loginfo("[%s] CAXST Data Written to File at: %s" % (rospy.Time.now(),xst_file))
    #     return

    # def log_pim_to_csv(self):
    #     """Log the preintegration data to a csv file"""
    #     if not self.preintegration_data:
    #         rospy.logwarn("[%s] No Preintegration Data" % rospy.Time.now())
    #         return
    #     else:
    #         preintegration_file = join("/ros/logs/", f"preintegration_data_{rospy.Time.now()}.csv")
    #         with open(preintegration_file, mode='w', newline='') as csvfile:
    #             writer = csv.writer(csvfile)
    #             # Write header
    #             writer.writerow(["ti","tj","key1", "key2", "position_x", "position_y", "position_z", "q_x", "q_y", "q_z", "q_w", "sig_x", "sig_y", "sig_z, sig_r, sig_p, sig_y"])
    #             # Write data rows
    #             for entry in self.preintegration_data:
    #                 # Check that there is data before attemptin to access the indices
    #                 if "position" not in entry or "orientation" not in entry or "sigmas" not in entry:
    #                     rospy.logwarn("[%s] Preintegration Data Entry Missing Fields: %s" % (rospy.Time.now(), entry))
    #                     continue
    #                 # Extract the position, orientation and sigmas
    #                 if not isinstance(entry["position"], (list, np.ndarray)) or not isinstance(entry["orientation"], (list, np.ndarray)) or not isinstance(entry["sigmas"], (list, np.ndarray)):
    #                     rospy.logwarn("[%s] Preintegration Data Entry Fields are not lists or arrays: %s" % (rospy.Time.now(), entry))
    #                     continue
    #                 if len(entry["position"]) != 3 or len(entry["orientation"]) != 4 or len(entry["sigmas"]) != 6:
    #                     rospy.logwarn("[%s] Preintegration Data Entry Fields have incorrect lengths: %s" % (rospy.Time.now(), entry))
    #                     continue
    #                 # Write the data
    #                 position = entry["position"]
    #                 orientation = entry["orientation"]
    #                 sigmas = entry["sigmas"]
    #                 writer.writerow([
    #                     entry["ti"].to_sec(),
    #                     entry["tj"].to_sec(),
    #                     entry["key1"],
    #                     entry["key2"],
    #                     position[0], position[1], position[2],
    #                     orientation[0], orientation[1], orientation[2], orientation[3],
    #                     sigmas[0], sigmas[1], sigmas[2], sigmas[3], sigmas[4], sigmas[5]
    #                 ])
    #         rospy.loginfo("[%s] Preintegration Data Written to File at: %s" % (rospy.Time.now(), preintegration_file))
    #     return

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