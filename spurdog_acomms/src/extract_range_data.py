#!/usr/bin/env python3
import rospy
import csv
from os.path import dirname, join, abspath
import numpy as np
from datetime import datetime
import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32

from spurdog_acomms_utils.nmea_utils import (
    parse_nmea_sentence,
    parse_nmea_cacmd,
    parse_nmea_cacma,
    parse_nmea_cacmr,
    parse_nmea_carfp,
    parse_nmea_cacst,
    parse_nmea_caxst,
    parse_nmea_carev
)

class CycleManager:
    """This is a node to run the comms cycle for the vehicle."""
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.landmarks = {"L0":[-74.5193539608157,-38.9298973079931,1.5], "L1":[66.5150726324041,25.969767675496275,1.5]} # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sound_speed = float(rospy.get_param("sound_speed", 1486))
        self.ping_method = None
        self.range_data = []
        self.cst_data = []
        self.xst_data = []
        self.partial_graph_data = {}
        self.cycle_graph_data = {}
        self.inbound_init_priors = {}
        self.inbound_partial_graphs = {}
        self.local_address = 0
        # Check services
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        # Start the cycle
        rospy.loginfo("[%s] Extractor Node Configured" % rospy.Time.now())
        rospy.sleep(1) # allow for modem to configure

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

        # elif nmea_type == "$CACMA": # Modem-to-host acknowledgement of a ping recieved
        #     src, dest, recieved_ping_time = parse_nmea_cacma(data)
        #     if data[1] == "PNG" and dest == self.local_address:
        #         self.request_preintegration(recieved_ping_time, True) # Request a relative pose measurement
        #         rospy.loginfo("[%s] Received Ping from %s" % (recieved_ping_time, chr(ord("A") + src)))
        #     elif data[1] == "PNG":
        #         rospy.loginfo("[%s] Overheard Ping from %s to %s" % (recieved_ping_time, chr(ord("A") + src), chr(ord("A") + dest)))
        #     else:
        #         rospy.logerr("[%s] Received $CACMA with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACMR": # Modem-to-host acknowledgement of a ping response
            src, dest, recieved_ping_time, owtt = parse_nmea_cacmr(data)
            if data[0] == "PNR" and src == self.local_address:
                rospy.loginfo("[%s] Received Ping Response from %s" % (recieved_ping_time, chr(ord("A") + dest)))
                ping_time = recieved_ping_time.to_sec() - owtt
                self.range_data.append([ping_time, src, dest, owtt, owtt*self.sound_speed])
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

        # elif nmea_type == "$CAREV" and self.ping_method == None: # Modem-to-host $CAREV message to determine the firmware version
        #     firmware_version = parse_nmea_carev(data)
        #     if firmware_version[0] == "3":
        #         # New deckbox firmware
        #         self.ping_method = "ping with payload"
        #     else:
        #         # Old deckbox firmware
        #         self.ping_method = "no payload"
        else:
            return
        return

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
        #log_dir = "/ros/logs/"
        log_dir = "/home/morrisjp/bags/June/06June/"
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
        cycle_mgr.summarize_range_data()
        rospy.loginfo("[%s] Comms Cycle Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Comms Cycle Mgr Exiting")
        exit(0)