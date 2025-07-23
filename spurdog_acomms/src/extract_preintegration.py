#!/usr/bin/env python3
import rospy
import csv
from os.path import dirname, join, abspath
import numpy as np
from datetime import datetime
import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped
from ros_acomms_msgs.msg import(
    TdmaStatus, QueueStatus, PingReply
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)
from spurdog_acomms.msg import(
    PoseFactorStamped, RangeFactorStamped
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
    """This is a specialized post-processing node to handle the bags from 27June which have some unique deficiencies.
    - Bags lack relative pose measurements (but have the ti/tj timestamps), so we request preintegration over these and publish a pose factor
    - Bags have range factors, but the key1 is off by one index, so we re-key the range factors to match the pose factors
    """
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.local_address = rospy.get_param("modem_address", 0)  # Local address of the vehicle, default is 0
        self.num_agents = rospy.get_param("num_agents", 1)
        self.num_landmarks = int(rospy.get_param("num_landmarks", 2))
        self.landmarks = {"L0":[-74.5193539608157,-38.9298973079931,1.5], "L1":[66.5150726324041,25.969767675496275,1.5]} # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = float(rospy.get_param("sound_speed", 1486))
        # Variables for addressing
        self.modem_addresses = {}
        self.cycle_target_mapping = {}
        # Variables for acomms event topic
        self.ping_method = "ping with payload"
        self.ping_timeout = 5 # seconds
        # Variables for external sensors
        self.gps_fix = [[1,2,3],[0,0,0,1],[1.7,1.7,3.5,0.1,0.1,0.1]] # [position, orientation, covariance]
        # Variables for message handling
        self.range_data = []
        self.cst_data = []
        self.xst_data = []
        self.preintegration_data = []
        self.nav_state_data = {}
        self.pose_time_lookup = {}
        self.in_water = False
        self.cov_analysis = {
            "var_x": [],
            "var_y": [],
            "var_z": [],
            "var_th": [],
            "var_phi": [],
            "var_psi": [],
            "cov_xy": [],
            "cov_xz": [],
            "cov_xth": [],
            "cov_xphi": [],
            "cov_xpsi": [],
            "cov_yz": [],
            "cov_yth": [],
            "cov_yphi": [],
            "cov_ypsi": [],
            "cov_zth": [],
            "cov_zphi": [],
            "cov_zpsi": [],
            "cov_thphi": [],
            "cov_thpsi": [],
            "cov_phipsi": [],
        }
        # Check services
        rospy.loginfo("[%s] Waiting for services..." % rospy.Time.now())
        rospy.wait_for_service("preintegrate_imu")
        self.preintegrate_imu = rospy.ServiceProxy("preintegrate_imu", PreintegrateImu)
        rospy.loginfo("[%s] Services ready, initializing topics" % rospy.Time.now())
        # Initialize topics
        # Establish the message subs and pubs
        #self.nav_state_sub = rospy.Subscriber("nav_state", PoseStamped, self.on_nav_state, queue_size=1)
        #self.acomms_event_pub = rospy.Publisher("led_command", String, queue_size=1)
        #self.range_logging_sub = rospy.Subscriber("modem/ping_reply",PingReply, self.on_ping_reply, queue_size=1)
        self.pose_factor_sub = rospy.Subscriber("pose_factor", PoseFactorStamped, self.on_pose_factor, queue_size=1)
        self.range_factor_sub = rospy.Subscriber("range_factor", RangeFactorStamped, self.on_range_factor, queue_size=1)
        # Initialize the factor publishers
        #self.range_factor_pub = rospy.Publisher("range_factor_recovered", RangeFactorStamped, queue_size=1)
        self.pose_factor_pub = rospy.Publisher("pose_factor_recovered", PoseFactorStamped, queue_size=1)
        #self.integrated_state_pub = rospy.Publisher("integrated_state_recovered", PoseWithCovarianceStamped, queue_size=1)
        #self.range_factor_pub = rospy.Publisher("range_factor", RangeFactorStamped, queue_size=1)
        # Initialize the modem addresses and cycle targets
        rospy.loginfo("[%s] Topics ready, initializing comms cycle" % rospy.Time.now())
        self.initial_ti = rospy.Time(0)  # Initial time for the cycle
        # Start the cycle
        self.configure_comms_cycle()
        rospy.loginfo("[%s] Comms Cycle Configured" % rospy.Time.now())
        rospy.sleep(10) # allow for modem to configure
        # Attempt a ping to the first target
        rospy.loginfo("[%s] Starting Comms Cycle" % rospy.Time.now())

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

    def on_pose_factor(self, msg: PoseFactorStamped):
        """This function handles the pose factor messages.
        It extracts the pose information and stores it in the preintegration data.
        """
        # Extract the timestamp, then call prientegration using this timestamp
        tj = msg.header.stamp
        # Check the pose factor key1 and key2, if they are for the local address, then we can proceed
        local_chr = chr(ord("A") + self.local_address)
        key1_chr = msg.key1[0]  # The first character is the local address
        key1_index = int(msg.key1[1:])  # The rest is the index
        if key1_chr != local_chr:
            rospy.logwarn(f"Pose factor key1 {msg.key1} does not match local address {local_chr}, skipping")
            return
        else:
            self.request_preintegration(tj, adv_pose=True)
        return

    def on_range_factor(self, msg: RangeFactorStamped):
        """This function handles the range factor messages.
        It extracts the range information and stores it in the range data.
        """
        # Re-key the range factor key1 (advance by 1)
        key1 = msg.key1
        local_chr = key1[0]  # The first character is the local address
        key1_index = int(key1[1:])  # The rest is the index
        key1_rekeyed = local_chr + str(key1_index + 1)  # Re-key the range factor key1
        # Update the key1 in the message
        msg.key1 = key1_rekeyed
        #self.range_factor_pub.publish(msg)
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
        ti = self.initial_ti
        if ti == tj:
            rospy.logwarn(f"Attempted to preintegrate at the same time {ti.to_sec()}, skipping preintegration.")
            return
        try:
            #rospy.loginfo(f"Attempting to preintegrate between {ti.to_sec()} and {tj.to_sec()}")
            response = self.preintegrate_imu(ti,tj)
            if adv_pose:
                # Advance the key indices and the time
                key1 = local_chr + str(key1_index)
                key2 = local_chr + str(key1_index+1)
                self.modem_addresses[local_chr][1] = key1_index + 1
                self.pose_time_lookup[key2] = tj
                #rospy.loginfo(f"Advancing pose from {key1} to {key2} at {tj.to_sec()}")
                x_ij = response.pose_delta
                # pose_delta is a PoseWithCovariance
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
                self.preintegration_data.append({
                    "ti": ti,
                    "tj": tj,
                    "key1": key1,
                    "key2": key2,
                    "position": position,
                    "orientation": orientation,
                    "sigmas": sigmas,
                })
                # Convetr orienation and dr_orientation to rpy
                orientation_rpy = spt.Rotation.from_quat(orientation).as_euler('xyz', degrees=True)
                # Log the preintegrated pose
                # Log thepreintegrated pose
                # rospy.loginfo(
                #     f"Preintegrated pose from {key1} to {key2} at {tj.to_sec():.2f}: "
                #     f"position={[f'{x:.4f}' for x in position]}, "
                #     f"orientation={[f'{r:.2f}' for r in orientation_rpy]},\n "
                #     f"sigmas={[f'{s:.4f}' for s in sigmas]}"
                # )
                # Publish the pose factor
                pose_factor_msg = PoseFactorStamped()
                pose_factor_msg.header.stamp = tj
                pose_factor_msg.header.frame_id = "modem"
                pose_factor_msg.key1 = key1
                pose_factor_msg.key2 = key2
                pose_factor_msg.pose = PoseWithCovariance()
                pose_factor_msg.pose.pose.position = Point(
                    x=x_ij.pose.pose.position.x,
                    y=x_ij.pose.pose.position.y,
                    z=x_ij.pose.pose.position.z
                )
                pose_factor_msg.pose.pose.orientation = Quaternion(
                    x=x_ij.pose.pose.orientation.x,
                    y=x_ij.pose.pose.orientation.y,
                    z=x_ij.pose.pose.orientation.z,
                    w=x_ij.pose.pose.orientation.w
                )
                pose_factor_msg.pose.covariance = np.array(x_ij.pose.covariance).reshape((6, 6)).flatten().tolist()
                # log the relative pose factor
                rospy.loginfo(f"Publishing pose factor from {key1} to {key2} at {tj.to_sec():.2f}: "
                                f"position={[f'{x:.4f}' for x in position]}, "
                                f"orientation={[f'{r:.2f}' for r in orientation_rpy]},\n "
                                f"sigmas={[f'{s:.4f}' for s in sigmas]}")

                self.pose_factor_pub.publish(pose_factor_msg)
                # Append results to the covariance analysis
                self.cov_analysis["var_x"].append(covariance[0, 0])
                self.cov_analysis["var_y"].append(covariance[1, 1])
                self.cov_analysis["var_z"].append(covariance[2, 2])
                self.cov_analysis["var_th"].append(covariance[3, 3])
                self.cov_analysis["var_phi"].append(covariance[4, 4])
                self.cov_analysis["var_psi"].append(covariance[5, 5])
                self.cov_analysis["cov_xy"].append(covariance[0, 1])
                self.cov_analysis["cov_xz"].append(covariance[0, 2])
                self.cov_analysis["cov_xth"].append(covariance[0, 3])
                self.cov_analysis["cov_xphi"].append(covariance[0, 4])
                self.cov_analysis["cov_xpsi"].append(covariance[0, 5])
                self.cov_analysis["cov_yz"].append(covariance[1, 2])
                self.cov_analysis["cov_yth"].append(covariance[1, 3])
                self.cov_analysis["cov_yphi"].append(covariance[1, 4])
                self.cov_analysis["cov_ypsi"].append(covariance[1, 5])
                self.cov_analysis["cov_zth"].append(covariance[2, 3])
                self.cov_analysis["cov_zphi"].append(covariance[2, 4])
                self.cov_analysis["cov_zpsi"].append(covariance[2, 5])
                self.cov_analysis["cov_thphi"].append(covariance[3, 4])
                self.cov_analysis["cov_thpsi"].append(covariance[3, 5])
                self.cov_analysis["cov_phipsi"].append(covariance[4, 5])
                #rospy.loginfo(f"Published pose factor: {pose_factor_msg}")
                # This allows for calling preintegration to clear the queue without advancing the pos
            self.initial_ti = tj  # Update the initial time to the current time
        except rospy.ServiceException as e:
            rospy.logerr(f"Preintegration failed: {response.error_message}") # Generate a placeholder response for comms testing
            response = PreintegrateImuResponse(
                pose_delta=PoseWithCovarianceStamped(
                    header=Header(
                        stamp=tj,
                        frame_id="imu"
                    ),
                    pose=PoseWithCovariance(
                        pose=Pose(
                            position=Point(x=0.0, y=0.0, z=0.0),
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                        ),
                        covariance= np.zeros((6, 6)).flatten().tolist()
                    )
                )
            )
        return

    def analyze_covariance(self):
        """This function analyzes the covariance data collected during the cycle.
        It computes the mean and standard deviation of the covariance values.
        """
        cov_analysis = {}
        for key, values in self.cov_analysis.items():
            if values:
                cov_analysis[key] = {
                    "mean": np.mean(values),
                    "std": np.std(values)
                }
            else:
                cov_analysis[key] = {
                    "mean": 0.0,
                    "std": 0.0
                }
        rospy.loginfo(f"Covariance Analysis: {cov_analysis}")
        return cov_analysis

    def run(self):
        """This function runs the comms cycle.
        """
        # Run the routine preintegration at a regular interval
        rate = rospy.Rate(0.33)
        while not rospy.is_shutdown():
            #self.routine_preintegration()
            rate.sleep()

if __name__ == "__main__":

    try:
        cycle_mgr = CycleManager()
        cycle_mgr.run()
    # When killed, run analyze_covariance and log the results
    except rospy.ROSException as e:
        rospy.logerr("[%s] Comms Cycle Mgr Exception: %s" % (rospy.Time.now(), e))
        cycle_mgr.analyze_covariance()
    finally:
        rospy.loginfo("[%s] Comms Cycle Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Comms Cycle Mgr Exiting")
        cycle_mgr.analyze_covariance()
        exit(0)