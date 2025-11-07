#!/usr/bin/env python3
import rospy
import csv
from os.path import dirname, join, abspath
import numpy as np
from datetime import datetime
import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped

from spurdog_acomms_utils.nmea_utils import (
    parse_nmea_sentence,
    parse_nmea_cacmd,
    parse_nmea_cacma,
    parse_nmea_cacmr,
    parse_nmea_carfp,
    parse_nmea_cacst,
    parse_nmea_caxst,
    parse_nmea_carev,
    parse_nmea_catxf,
    # parse_nmea_catxp,
)
from spurdog_acomms_utils.param_utils import get_namespace_param

class CycleManager:
    """This is a node to run the comms cycle for the vehicle.
    It will handle the ping requests, responses, and the range data collection for a ping-only cycle.
    It is untested on ping + data cycles, but should work with minimal changes."""
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        # Prefer landmarks via ROS params; fall back to legacy literals for safety
        # Previous hard-coded literal preserved for reference:
        # self.landmarks = {"L0":[-74.5193539608157,-38.9298973079931,1.5], "L1":[66.5150726324041,25.969767675496275,1.5]}
        self.landmarks = get_namespace_param("landmarks", {
            "L0": [-74.5193539608157, -38.9298973079931, 1.5],
            "L1": [66.5150726324041, 25.969767675496275, 1.5]
        }, warn_if_missing=True)
        self.sound_speed = float(rospy.get_param("sound_speed", 1486))
        self.ping_method = None
        self.range_data = []
        self.ping_data = {
            "timestamp": None,
            "time_CACMD": None,
            "time_CAXST": None,
            "time_CACST": None,
            "time_CACMR": None,
            "CACMD-CAXST": None,
            "CAXST-CACST": None,
            "CACST-CACMR": None,
            "CACMD-CACMR": None,
            "SRC": None,
            "DEST": None,
            "OWTT": None,
            "RANGE": None,
            "DOP": None,
            "RSS": None,
            "SNR-IN": None,
            "SNR-OUT": None,
        }
        self.cst_data = []
        self.xst_data = []
        self.partial_graph_data = {}
        self.cycle_graph_data = {}
        self.inbound_init_priors = {}
        self.inbound_partial_graphs = {}
        self.local_address = 0
        self.time_offset = None
        # Check services
        # Monitor NMEA messages to track the pings and trigger relative pose measurements
        self.nmea_from_modem = rospy.Subscriber("modem/nmea_from_modem", String, self.on_nmea_from_modem)
        self.nav_state = rospy.Subscriber("nav_state", PoseStamped, self.on_nav) # GPS messages are also NMEA messages
        # Start the cycle
        rospy.loginfo("[%s] Extractor Node Configured" % rospy.Time.now())
        rospy.sleep(1) # allow for modem to configure

    # Ping Handling:
    def on_nav(self, msg):
        # Get the timestamp from the Imu message ()
        current_time = msg.header.stamp.to_sec()
        if self.time_offset is None:
            # Set the time offset to the current time
            self.time_offset = rospy.Time.now().to_sec() - current_time
        else:
            # Update the timestamp with the time offset
            pass
        return

    def on_nmea_from_modem(self, msg):
        """This function receives NMEA messages from the modem
        """
        nmea_type, data = parse_nmea_sentence(msg.data)
        # Process the NMEA data by field
        if nmea_type == "$CACMD": # Modem-to-host acknowledgement of a ping command
            src, dest, commanded_ping_time = parse_nmea_cacmd(data)
            commanded_ping_time = rospy.Time(commanded_ping_time.to_sec() - self.time_offset if self.time_offset is not None else commanded_ping_time.to_sec())
            # If ping data is entirely empty don't append it to the range data
            if self.ping_data["SRC"] is not None and self.ping_data["DEST"] is not None:
                if self.ping_data["timestamp"] is None and self.ping_data["time_CACMD"] is not None:
                    # Average the commanded ping time and the time_CACMD in the ping data (which are both rostime.Time)
                    self.ping_data["timestamp"] = rospy.Time((commanded_ping_time.to_sec() + self.ping_data["time_CACMD"])/ 2.0).to_sec()
                else:
                    pass
                self.range_data.append([
                    # Append all fields of ping data to the range data
                    self.ping_data["timestamp"],
                    self.ping_data["time_CACMD"],
                    self.ping_data["time_CAXST"],
                    self.ping_data["time_CACST"],
                    self.ping_data["time_CACMR"],
                    self.ping_data["CACMD-CAXST"],
                    self.ping_data["CAXST-CACST"],
                    self.ping_data["CACST-CACMR"],
                    self.ping_data["CACMD-CACMR"],
                    self.ping_data["SRC"],
                    self.ping_data["DEST"],
                    self.ping_data["OWTT"],
                    self.ping_data["RANGE"],
                    self.ping_data["DOP"],
                    self.ping_data["RSS"],
                    self.ping_data["SNR-IN"],
                    self.ping_data["SNR-OUT"]
                ])
            else:
                rospy.logwarn("[%s] No previous ping data to append to range data" % rospy.Time.now())
            # Reset the ping data for the next ping
            self.ping_data = {
                "timestamp": None,
                "time_CACMD": commanded_ping_time.to_sec(),
                "time_CAXST": None,
                "time_CACST": None,
                "time_CACMR": None,
                "CACMD-CAXST": None,
                "CAXST-CACST": None,
                "CACST-CACMR": None,
                "CACMD-CACMR": None,
                "SRC": src,
                "DEST": dest,
                "OWTT": None,
                "RANGE": None,
                "DOP": None,
                "RSS": None,
                "SNR-IN": None,
                "SNR-OUT": None
            }
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
            # Check if we have a ping data entry for this ping
            if self.ping_data["SRC"] is not None and self.ping_data["DEST"] is not None:
                if self.ping_data["SRC"] == src and self.ping_data["DEST"] == dest:
                    # Update the ping data with the owtt
                    self.ping_data["time_CACMR"] = recieved_ping_time
                    if self.ping_data["time_CACST"] is not None:
                        self.ping_data["CACST-CACMR"] = np.round(recieved_ping_time - self.ping_data["time_CACST"], 6)
                    self.ping_data["CACMD-CACMR"] = np.round(recieved_ping_time - self.ping_data["time_CACMD"], 6)
                    self.ping_data["OWTT"] = owtt
                    self.ping_data["RANGE"] = np.round(owtt * self.sound_speed, 6)
                    self.ping_data["timestamp"] = ping_time = recieved_ping_time - owtt
            else:
                rospy.logerr("[%s] Received $CACMR with unexpected data: %s" % (rospy.Time.now(), data))

        elif nmea_type == "$CACST": # Modem-to-host report of signal recieved
            cst_statistics = parse_nmea_cacst(data)
            # Check if we have a ping data entry for this ping
            if self.ping_data["SRC"] is not None and self.ping_data["DEST"] is not None:
                if self.ping_data["SRC"] == cst_statistics[2] and self.ping_data["DEST"] == cst_statistics[1]:
                    # Update the ping data with the cst statistics
                    self.ping_data["time_CACST"] = cst_statistics[0]
                    if self.ping_data["time_CACMR"] is not None:
                        self.ping_data["CACST-CACMR"] = np.round(cst_statistics[0] - self.ping_data["time_CACMR"], 6)
                    self.ping_data["CAXST-CACST"] = np.round(cst_statistics[0] - self.ping_data["time_CAXST"],6)
                    self.ping_data["DOP"] = cst_statistics[8]
                    self.ping_data["RSS"] = cst_statistics[9]
                    self.ping_data["SNR-IN"] = cst_statistics[6]
                    self.ping_data["SNR-OUT"] = cst_statistics[7]
                else:
                    # Reset the ping data
                    rospy.logerr("[%s] Received $CACST with unexpected SRC/DEST: %s/%s" % (rospy.Time.now(), cst_statistics[2], cst_statistics[1]))
                    self.ping_data["timestamp"] = rospy.Time((cst_statistics[0].to_sec() + self.ping_data["time_CACMD"])/ 2.0).to_sec()
                    self.range_data.append([
                        # Append all fields of ping data to the range data
                        self.ping_data["timestamp"],
                        self.ping_data["time_CACMD"],
                        self.ping_data["time_CAXST"],
                        self.ping_data["time_CACST"],
                        self.ping_data["time_CACMR"],
                        self.ping_data["CACMD-CAXST"],
                        self.ping_data["CAXST-CACST"],
                        self.ping_data["CACST-CACMR"],
                        self.ping_data["CACMD-CACMR"],
                        self.ping_data["SRC"],
                        self.ping_data["DEST"],
                        self.ping_data["OWTT"],
                        self.ping_data["RANGE"],
                        self.ping_data["DOP"],
                        self.ping_data["RSS"],
                        self.ping_data["SNR-IN"],
                        self.ping_data["SNR-OUT"]
                    ])
            self.cst_data.append(cst_statistics)

        elif nmea_type == "$CAXST": # Modem-to-host report of signal transmitted
            xst_statistics = parse_nmea_caxst(data)
            # Check if we have a ping data entry for this ping
            if self.ping_data["SRC"] is not None and self.ping_data["DEST"] is not None:
                if self.ping_data["SRC"] == xst_statistics[1] and self.ping_data["DEST"] == xst_statistics[2]:
                    # Update the ping data with the xst statistics
                    self.ping_data["time_CAXST"] = xst_statistics[0]
                    self.ping_data["CACMD-CAXST"] = np.round(xst_statistics[0] - self.ping_data["time_CACMD"],6)
            self.xst_data.append(xst_statistics)
        elif nmea_type == "$CATXP": # Modem-to-host report of beginning transmission
            # This is a transmit report, we can ignore it for now
            #rospy.loginfo("[%s] Received $CATXP message: %s" % (rospy.Time.now(), data))
            pass
        else:
            return
        return

    # def summarize_range_data(self):
    #     """This function summarizes the range data collected upon node shutdown"""
    #     rospy.loginfo("[%s] Range Data Summary:" % rospy.Time.now())

    #     # Report the number of completed ranges to each dest
    #     range_summary = {}
    #     for entry in self.range_data:
    #         timestamp, src, dest, owtt, measured_range = entry
    #         if dest not in range_summary:
    #             range_summary[dest] = []
    #         range_summary[dest].append((timestamp, src, owtt, measured_range))
    #     for dest in range_summary.keys():
    #         # if there are no ranges to this dest, skip it
    #         if not range_summary[dest]:
    #             rospy.loginfo("[%s] No ranges to %s" % (rospy.Time.now(), chr(ord("A") + dest)))
    #         else:
    #             #Check if there is an entry that includes a measured_range
    #             has_measured_range = any(r[3] is not None for r in range_summary[dest])
    #             if not has_measured_range:
    #                 rospy.loginfo("[%s] Ranges to %s: %d / %d valid" % (rospy.Time.now(), chr(ord("A") + dest), 0, len(range_summary[dest])))
    #             else:
    #                 # Extract valid entries (measured_range not None)
    #                 valid_range_set = [r for r in range_summary[dest] if r[3] is not None]

    #                 # Extract only the ranges and timestamps
    #                 valid_ranges = [r[3] for r in valid_range_set]
    #                 valid_timestamps = [r[0] for r in valid_range_set]

    #                 # Compute statistics
    #                 num_valid = len(valid_ranges)
    #                 min_range = np.min(valid_ranges)
    #                 max_range = np.max(valid_ranges)
    #                 mean_range = np.mean(valid_ranges)
    #                 std_range = np.std(valid_ranges)

    #                 # Compute time delta (assumes timestamps are rospy.Time or float)
    #                 time_delta = valid_timestamps[-1] - valid_timestamps[0]

    #                 # If rospy.Time, convert to seconds
    #                 if hasattr(time_delta, 'to_sec'):
    #                     time_delta_sec = time_delta.to_sec()
    #                 else:
    #                     time_delta_sec = time_delta  # already float

    #                 # Log or return as needed
    #                 print(f"Valid Ranges: {num_valid}")
    #                 print(f"Min: {min_range}, Max: {max_range}, Mean: {mean_range}, Std Dev: {std_range}")
    #                 print(f"Time Span: {time_delta_sec} seconds")
    #                 rospy.loginfo("[%s] Ranges to %s: %d / %d valid, From %.2f - %.2fm, Mean: %.2fm, dT: %.2fsec" % (rospy.Time.now(), chr(ord("A") + dest), num_valid, len(range_summary[dest]), min_range, max_range, mean_range, time_delta_sec))
    #     return

    def summarize_range_data(self):
        """This function summarizes the range data collected upon node shutdown"""
        rospy.loginfo("[%s] Range Data Summary:" % rospy.Time.now())

        # Report the number of completed ranges to each dest
        range_summary = {}
        for entry in self.range_data:
            timestamp, time_CACMD, time_CAXST, time_CACST, time_CACMR, CACMD_CAXST, CAXST_CACST, CACST_CACMR, CACMD_CACMR, src, dest, owtt, measured_range, dop, rss, snr_in, snr_out = entry
            if dest not in range_summary:
                range_summary[dest] = []
            range_summary[dest].append((timestamp, src, owtt, measured_range))
        # Iterate through each destination and summarize the ranges
        rospy.loginfo("[%s] Total Ranges Collected: %d" % (rospy.Time.now(), len(self.range_data)))
        for dest in range_summary.keys():
            # if there are no ranges to this dest, skip it
            if not range_summary[dest]:
                rospy.loginfo("[%s] No ranges to %s" % (rospy.Time.now(), chr(ord("A") + dest)))
                continue

            # Extract valid entries (measured_range not None)
            valid_range_set = [r for r in range_summary[dest] if r[3] is not None]

            if not valid_range_set:
                rospy.loginfo("[%s] Ranges to %s: 0 / %d valid" % (rospy.Time.now(), chr(ord("A") + dest), len(range_summary[dest])))
                continue
            # Extract only the ranges and timestamps
            valid_ranges = [r[3] for r in valid_range_set]
            valid_timestamps = [r[0] for r in valid_range_set]
            # Compute statistics
            num_valid = len(valid_ranges)
            min_range = np.min(valid_ranges)
            max_range = np.max(valid_ranges)
            mean_range = np.mean(valid_ranges)
            std_range = np.std(valid_ranges)
            # Log the statistics of the valid ranges
            rospy.loginfo("[%s] Ranges to %s: %d / %d valid, From %.2f - %.2fm" % (
                rospy.Time.now(), chr(ord("A") + dest), num_valid, len(range_summary[dest]),
                min_range, max_range
            ))

    def log_ranges_to_csv(self):
        """ Log ranges, cst and xst data to csv"""
        if not self.xst_data:
            rospy.logwarn("[%s] No Signals Transmitted" % rospy.Time.now())
            return
        else:
            xst_timestamp = self.xst_data[0][0]
            # Convert from sec_to_nanosec
            if isinstance(xst_timestamp, rospy.Time):
                xst_timestamp = xst_timestamp.to_nsec()
            elif isinstance(xst_timestamp, float):
                xst_timestamp = int(xst_timestamp * 1e9)
            else:
                rospy.logerr("[%s] Invalid timestamp type: %s" % (rospy.Time.now(), type(xst_timestamp)))
                return

        # Create the csv files:
        #log_dir = "/ros/logs/"
        log_dir = "/home/morrisjp/bags/June/15June/"
        range_file = join(log_dir, f"range_data_{xst_timestamp}.csv")
        cst_file = join(log_dir, f"cst_data_{xst_timestamp}.csv")
        xst_file = join(log_dir, f"xst_data_{xst_timestamp}.csv")
        with open(range_file, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp", "time_CACMD", "time_CAXST", "time_CACST", "time_CACMR", "CACMD-CAXST", "CAXST-CACST", "CACST-CACMR", "CACMD-CACMR", "src", "dest", "owtt", "measured_range", "dop(m/s)", "rss(dB)", "snr_in(dB)", "snr_out(dB)"])
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