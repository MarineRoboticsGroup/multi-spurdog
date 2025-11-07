#!/usr/bin/env python3
import rospy
from os.path import dirname, join, abspath
import numpy as np
from datetime import datetime
import scipy.spatial.transform as spt
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)

from spurdog_acomms_utils.param_utils import get_namespace_param
from spurdog_acomms_utils.landmark_utils import get_landmark_pos, validate_landmarks


class PingTest:
    """
    This node is for running basic static ping tests between modems.
    - It also may be used to test modem params between a running AUV and fixed landmarks.
    - It sends regular pings to a list of possible destinations
    - It collects only the one way travel time (OWTT) and calculates the range
    - It assumes that the ground-truth range between the modem locations is known
    - It should be run with rosbag logging to capture the additional ping data
      in the modem-to-host messages.
    """

    def __init__(self):
        # Initialize the node
        rospy.init_node('ping_test_manager', anonymous=True)
        # Use the param helper to fetch landmarks (works when run in different namespaces)
        self.landmarks = get_namespace_param("landmarks", {}, warn_if_missing=True)
        try:
            validate_landmarks(self.landmarks)
        except Exception:
            rospy.logwarn("[ping_test] landmarks param missing or malformed; continuing with empty set")
        self.sound_speed = rospy.get_param("~sound_speed", 1500) # m/s
        self.ping_timeout = rospy.get_param("~ping_timeout", 5) # seconds
        self.pings_to_attempt = rospy.get_param("~pings_to_attempt", 100)
        self.ping_sequence = 0
        self.expected_range = None
        self.ping_targets = self.get_targets()
        self.range_data = []
        # Check for ping services
        rospy.loginfo("[%s] Waiting for services..." % rospy.Time.now())
        rospy.wait_for_service("modem/ping_modem")
        self.ping_client = rospy.ServiceProxy("modem/ping_modem", PingModem)
        rospy.sleep(10) # Wait for the modem to be ready before trying to ping
        rospy.loginfo("[%s] Ping Test Manager Initialized" % rospy.Time.now())

    def get_targets(self):
        """Get the list of target addresses from the landmarks parameter."""
        if not self.landmarks:
            rospy.logwarn("[%s] No landmarks found in parameters." % rospy.Time.now())
            return []
        targets = []
        # targets should be a list of integers starting from 1 and increasing to N+1 for N landmarks
        for i in range(1, len(self.landmarks) + 1):
            targets.append(i)
        rospy.loginfo("[%s] Found %d landmarks: %s" % (rospy.Time.now(), len(targets), targets))
        return targets

    def get_expected_range(self):
        # Get the cartesian range between the two landmarks
        """Calculate the expected range between two landmarks."""
        if len(self.landmarks) < 2:
            rospy.logwarn("[%s] Not enough landmarks to calculate range." % rospy.Time.now())
            return None
        # Get the first two landmarks (use safe accessor)
        landmark_keys = list(self.landmarks.keys())
        if len(landmark_keys) < 2:
            rospy.logwarn("[%s] Not enough landmarks to calculate range." % rospy.Time.now())
            return None
        k1, k2 = landmark_keys[0], landmark_keys[1]
        l1 = get_landmark_pos(self.landmarks, k1)
        l2 = get_landmark_pos(self.landmarks, k2)
        if l1 is None or l2 is None:
            rospy.logwarn("[%s] Landmark positions missing for %s or %s" % (rospy.Time.now(), k1, k2))
            return None
        # Calculate the Euclidean distance between the two landmarks
        self.expected_range = np.linalg.norm(np.array(l1) - np.array(l2))
        rospy.loginfo("[%s] Expected Range between %s and %s: %.4f meters" % (rospy.Time.now(), k1, k2, self.expected_range))
        return

    def run_ping_test(self):
        """Run the ping test."""
        rospy.loginfo("[%s] Running ping test" % rospy.Time.now())
        while self.ping_sequence <= self.pings_to_attempt:
            # Get the target (round robin
            target = self.ping_targets[self.ping_sequence % len(self.ping_targets)]
            self.send_ping(target)
            rospy.sleep(1) # prevents transmitting two pings too close together

        # After the test, summarize the results
        self.summarize_range_data()

    def send_ping(self, dest):
        """This function sends a ping to the modem
        Args:
            target_addr (int): the target address
            symbol (str): the local key "A1" to put in the payload
        """
        # Set the ping request parameters
        ping_req = PingModemRequest()
        ping_req.dest = dest
        ping_req.rate = 1
        ping_req.cdr = 0
        ping_req.timeout_sec = self.ping_timeout

        # Attempt the ping (will hold the while loop until timeout)
        try:
            ping_resp = self.ping_client(ping_req)

            if ping_resp.timed_out:
                self.range_data.append([rospy.Time.now().to_sec() -self.ping_timeout, 0, dest, None, None])
                rospy.logwarn("[%s] Ping %s Timed Out" % (rospy.Time.now(), self.ping_sequence))
            else:
                # Unpack the ping response
                dest = ping_resp.cst.src
                src = ping_resp.cst.dest
                owtt = ping_resp.one_way_travel_time
                tat = ping_resp.tat
                measured_range = owtt * self.sound_speed
                timestamp = ping_resp.cst.toa - rospy.Duration.from_sec(owtt)
                # Log all the fields
                self.range_data.append([timestamp.to_sec(), src, dest, owtt, measured_range])
                rospy.loginfo("[%s] Ping Successful: timestamp=%s, src=%d, dest=%d, owtt=%.4f, tat= %.4f, measured_range=%.4f" % (rospy.Time.now(), timestamp, src, dest, owtt, tat, measured_range))
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
        # Split by the dest address (3rd column) (i.e. all pings to landmark 1, all pings to landmark 2, etc.)
        unique_dests = np.unique(range_data_np[:,2])
        # Build a dictonary of responses for each dest
        dest_dict = {}
        for dest in unique_dests:
            dest_dict[dest] = range_data_np[range_data_np[:,2] == dest]
            successful_pings = dest_dict[dest][dest_dict[dest][:,3] > 0]
            success_rate = len(successful_pings) / len(dest_dict[dest]) * 100
            mean_owtt = np.mean(successful_pings[:,3])
            mean_range = np.mean(successful_pings[:, 4])
            max_range = np.max(successful_pings[:, 4])
            min_range = np.min(successful_pings[:, 4])
            std_range = np.std(successful_pings[:, 4])
            # Display a nice stats table of the results
            rospy.loginfo("---- Ping Statistics for Modem L%d ----" % int(dest-1))
            rospy.loginfo("Completed %d/%d pings (%.2f%)" % (len(successful_pings), len(dest_dict[dest]), success_rate))
            rospy.loginfo("Meas. Range (m): Min: %.4f, Mean: %.4f, Max: %.4f" % (min_range, mean_range, max_range))
            rospy.loginfo("Std. Dev. of Range: %.4f m" % (std_range))
        return

if __name__ == "__main__":

    try:
        test_mgr = PingTest()
        test_mgr.run_ping_test()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[%s] Ping Test Mgr Interrupted" % rospy.Time.now())
    except Exception as e:
        rospy.logerr("[%s] Ping Test Mgr Error: %s" % (rospy.Time.now(), e))
    finally:
        rospy.loginfo("[%s] Ping Test Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Ping Test Mgr Exiting")
        exit(0)