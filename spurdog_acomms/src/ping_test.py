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

class PingTest:
    """This is a node to run the comms cycle for the vehicle."""
    def __init__(self):
        rospy.init_node('ping_test_manager', anonymous=True)
        self.landmarks = {"L0":[-71.7845,-39.6078,1.5],
                          "L1":[65.0832,25.6598,1.5],
        }
        # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = 1500
        self.ping_timeout = 5 # seconds
        self.ping_sequence = 0
        self.pings_to_attempt = 100
        self.expected_range = None
        self.range_data = []
        # Check services
        rospy.loginfo("[%s] Waiting for services..." % rospy.Time.now())
        rospy.wait_for_service("modem/ping_modem")
        self.ping_client = rospy.ServiceProxy("modem/ping_modem", PingModem)
        # Initialize the modem addresses and cycle targets
        rospy.sleep(10) # allow for modem to configure
        rospy.loginfo("[%s] Ping Test Manager Initialized" % rospy.Time.now())

    def get_expected_range(self):
        # Get the cartesian range between the two landmarks
        """Calculate the expected range between two landmarks."""
        if len(self.landmarks) < 2:
            rospy.logwarn("[%s] Not enough landmarks to calculate range." % rospy.Time.now())
            return None
        # Get the first two landmarks
        landmark_keys = list(self.landmarks.keys())
        if len(landmark_keys) < 2:
            rospy.logwarn("[%s] Not enough landmarks to calculate range." % rospy.Time.now())
            return None
        landmark1 = self.landmarks[landmark_keys[0]]
        landmark2 = self.landmarks[landmark_keys[1]]
        # Calculate the Euclidean distance between the two landmarks
        self.expected_range = np.linalg.norm(np.array(landmark1) - np.array(landmark2))
        rospy.loginfo("[%s] Expected Range between %s and %s: %.4f meters" % (rospy.Time.now(), landmark_keys[0], landmark_keys[1], expected_range))
        return

    def run_ping_test(self):
        """Run the ping test."""
        rospy.loginfo("[%s] Running ping test" % rospy.Time.now())
        # Start the ping test
        while self.ping_sequence <= self.pings_to_attempt:
            self.send_ping()
            rospy.sleep(1)
        # Summarize the range data collected
        self.summarize_range_data()

    def send_ping(self):
        """This function sends a ping to the modem
        Args:
            target_addr (int): the target address
            symbol (str): the local key "A1" to put in the payload
        """
        # Set the ping request parameters
        ping_req = PingModemRequest()
        ping_req.dest = 1
        ping_req.rate = 1
        ping_req.cdr = 0
        ping_req.timeout_sec = self.ping_timeout

        # Attempt the ping:
        try:
            #rospy.loginfo("[%s] One Ping Only Vasily." % (rospy.Time.now()))
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
                rospy.logwarn("[%s] Ping Timed Out" % (rospy.Time.now()))
                self.range_data.append([rospy.Time.now().to_sec() -self.ping_timeout, 0, 1, None, None])
            else:
                rospy.loginfo("[%s] Ping %s Successful: "% (rospy.Time.now(), self.ping_sequence))
                dest = ping_resp.cst.src
                src = ping_resp.cst.dest
                owtt = ping_resp.one_way_travel_time
                tat = ping_resp.tat
                measured_range = owtt * self.sound_speed
                timestamp = ping_resp.cst.toa - rospy.Duration.from_sec(owtt)
                # Log all the fields
                rospy.loginfo("[%s] Ping Response: timestamp=%s, src=%d, dest=%d, owtt=%.4f, tat= %.4f, measured_range=%.4f" % (rospy.Time.now(), timestamp, src, dest, owtt, tat, measured_range))
                # Log the ping response
                self.range_data.append([timestamp.to_sec(), src, dest, owtt, measured_range])
            self.ping_sequence += 1
        except rospy.ServiceException as e:
            rospy.logerr("[%s] Ping Service Call Failed: %s" % (rospy.Time.now(), e))
        return

    def summarize_range_data(self):
        """Summarize the range data collected."""
        rospy.loginfo("[%s] Summarizing Range Data" % rospy.Time.now())
        if not self.range_data:
            rospy.logwarn("[%s] No range data collected." % rospy.Time.now())
            return

        # Convert to numpy array for easier processing
        range_data_np = np.array(self.range_data)
        # Print the summary
        rospy.loginfo("[%s] Range Data Summary:" % rospy.Time.now())
        # Print the success rate, mean, and standard deviation of the measured ranges
        successful_pings = range_data_np[range_data_np[:, 3] != None]
        if successful_pings.size == 0:
            rospy.logwarn("[%s] No successful pings." % rospy.Time.now())
            return
        success_rate = len(successful_pings) / len(range_data_np) * 100
        mean_owtt = np.mean(successful_pings[:,3])
        mean_range = np.mean(successful_pings[:, 4])
        std_range = np.std(successful_pings[:, 4])
        rospy.loginfo("[%s] %s / %s pings, Success Rate: %.2f%%" % (rospy.Time.now(), len(successful_pings), len(range_data_np), success_rate))
        rospy.loginfo("[%s] Mean Measured OWTT: %.4f sec" % (rospy.Time.now(), mean_owtt))
        rospy.loginfo("[%s] Mean Measured Range: %.4f meters" % (rospy.Time.now(), mean_range))
        rospy.loginfo("[%s] Standard Deviation of Measured Range: %.4f meters" % (rospy.Time.now(), std_range))
        # Print the expected range
        if self.expected_range is not None:
            rospy.loginfo("[%s] Expected Range: %.4f meters" % (rospy.Time.now(), self.expected_range))
            inferred_sound_speed = 1500 * (self.expected_range / mean_range)
            rospy.loginfo("[%s] Inferred Sound Speed: %.4f m/s" % (rospy.Time.now(), inferred_sound_speed))
        else:
            rospy.logwarn("[%s] Expected range not calculated." % rospy.Time.now())

if __name__ == "__main__":

    try:
        test_mgr = PingTest()
        test_mgr.run_ping_test()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[%s] Comms Test Mgr Interrupted" % rospy.Time.now())
    except Exception as e:
        rospy.logerr("[%s] Comms Test Mgr Error: %s" % (rospy.Time.now(), e))
    finally:
        rospy.loginfo("[%s] Comms Test Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Comms Test Mgr Exiting")
        exit(0)