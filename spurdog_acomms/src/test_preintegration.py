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
    TdmaStatus, QueueStatus
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
    """This is a node to run the comms cycle for the vehicle.
        - This is a lightweight version to test simple message reciept rates
        - This is configured as a bang-bang LBL controller (1 agent, 2 landmarks, no tdma)
    """
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.local_address = 0
        self.num_agents = 1
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
        # Check services
        rospy.loginfo("[%s] Waiting for services..." % rospy.Time.now())
        rospy.wait_for_service("preintegrate_imu")
        self.preintegrate_imu = rospy.ServiceProxy("preintegrate_imu", PreintegrateImu)
        rospy.loginfo("[%s] Services ready, initializing topics" % rospy.Time.now())
        # Initialize topics
        # Establish the message subs and pubs
        self.nav_state_sub = rospy.Subscriber("nav_state", PoseStamped, self.on_nav_state, queue_size=1)
        self.acomms_event_pub = rospy.Publisher("led_command", String, queue_size=1)
        # Initialize the factor publishers
        self.pose_factor_pub = rospy.Publisher("pose_factor", PoseFactorStamped, queue_size=1)
        self.range_factor_pub = rospy.Publisher("range_factor", RangeFactorStamped, queue_size=1)
        # Initialize the modem addresses and cycle targets
        rospy.loginfo("[%s] Topics ready, initializing comms cycle" % rospy.Time.now())
        self.configure_comms_cycle()
        self.initial_ti = rospy.Time(0)  # Initial time for the cycle
        self.nav_state_time = rospy.Time.now()  # Initialize the nav state time
        # Start the cycle
        rospy.loginfo("[%s] Comms Cycle Configured" % rospy.Time.now())
        rospy.sleep(10) # allow for modem to configure
        # Attempt a ping to the first target
        rospy.loginfo("[%s] Starting Comms Cycle" % rospy.Time.now())

    def configure_comms_cycle(self):
        """This function configures the comms cycle for the vehicle.
        """
        # Get the modem addresses and cycle targets
        self.modem_addresses = configure_modem_addresses(self.num_agents, self.num_landmarks, self.local_address)
        # Static target mapping
        self.cycle_target_mapping = {"0": [self.modem_addresses["L0"][0], self.modem_addresses["L1"][0]],}
        # self.cycle_target_mapping = configure_cycle_targets(self.modem_addresses)
        # Confifure the first pose in pose_time_lookup
        local_chr = chr(ord("A") + self.local_address)
        self.pose_time_lookup[local_chr + str(0)] = rospy.Time.now()  # Initial pose at time of cycle start
        # Print cycle targets for debugging
        rospy.loginfo("[%s] Cycle Targets: %s" % (rospy.Time.now(), self.cycle_target_mapping))
        return
    def on_nav_state(self, msg: PoseStamped):
        """This function is called when a new nav state message is received.
        It updates the in_water state and the pose time lookup.
        Args:
            msg (PoseStamped): The new nav state message
        """
        # Update the in_water state
        self.nav_state_time = msg.header.stamp
        if self.initial_ti == rospy.Time(0):
            self.initial_ti = self.nav_state_time
        # Log the nav state data
        self.nav_state_data[self.nav_state_time] = {
            "position": np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
            "orientation": np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]),
        }
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
                self.pose_factor_pub.publish(pose_factor_msg)
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

    def routine_preintegration(self):
        """This function is called to preintegrate the imu data at a regular interval.
        """
        # Call the preintegration service
        self.request_preintegration(self.nav_state_time, adv_pose=True)

        return

    def run(self):
        """This function runs the comms cycle.
        """
        # Run the routine preintegration at a regular interval
        rate = rospy.Rate(0.33)
        while not rospy.is_shutdown():
            self.routine_preintegration()
            rate.sleep()

if __name__ == "__main__":

    try:
        cycle_mgr = CycleManager()
        cycle_mgr.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[%s] Comms Cycle Mgr Interrupted" % rospy.Time.now())
    except Exception as e:
        rospy.logerr("[%s] Comms Cycle Mgr Error: %s" % (rospy.Time.now(), e))
    finally:
        rospy.loginfo("[%s] Comms Cycle Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Comms Cycle Mgr Exiting")
        exit(0)