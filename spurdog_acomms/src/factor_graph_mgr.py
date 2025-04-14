#!/usr/bin/env python3
import rospy
import numpy as np
from datetime import datetime
import scipy.spatial.transform as spt
from std_msgs.msg import Header, String, Time, Float32
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
from ros_acomms_msgs.msg import(
    TdmaStatus
)
from ros_acomms_msgs.srv import(
    PingModem, PingModemResponse, PingModemRequest
)
from spurdog_acomms.msg import(
    InitPrior, PartialGraph, PoseWithAssoc, RangeWithAssoc, CycleGraph, CommsCycleStatus
)
from spurdog_acomms.srv import(
    PreintegrateIMU, PreintegrateIMUResponse
)
from spurdog_acomms_utils.setup_utils import(
    configure_modem_addresses,
    configure_cycle_targets
)
from spurdog_acomms_utils.codec_utils import (
    decode_init_prior_data_from_int,
    decode_partial_graph_pose_from_int,
    decode_partial_graph_range_from_int,
)
from spurdog_acomms_utils.graph_utils import (
    marginalize_partial_graph
)

class GraphManager:
    """This is a node to run the comms cycle for the vehicle."""
    def __init__(self):
        rospy.init_node('comms_cycle_manager', anonymous=True)
        self.local_address = rospy.get_param("~modem_address", 0)
        self.num_agents = rospy.get_param("~num_agents", 1)
        self.num_landmarks = rospy.get_param("~num_landmarks", 0)
        self.landmarks = rospy.get_param("~landmarks", {}) # Assumes a dictionary of landmark positions {L1:[x,y,z], L2:[x,y,z], ...}
        self.sigma_range = rospy.get_param("~sigma_range", 0.1)
        # Variables for addressing
        self.modem_addresses = {}
        self.cycle_target_mapping = {}
        # Variables for tdma and cycle execution
        self.tdma_status = TdmaStatus()
        self.tdma_cycle_sequence = 0
        self.cycle_graph_data = {}
        self.inbound_init_priors = {}
        self.inbound_partial_graphs = {}
        self.in_water = False
        self.init_complete = False
        self.cycle_reset = False
        # Initialize topics
        self.tdma_from_modem = rospy.Subscriber("modem/tdma_status", TdmaStatus, self.on_tdma_status, queue_size=1)
        self.init_prior_sub = rospy.Subscriber("modem/from_acomms/init_prior", InitPrior, self.on_init_prior)
        self.partial_graph_sub = rospy.Subscriber("modem/from_acomms/partial_graph", PartialGraph, self.on_partial_graph)
        self.mission_status = rospy.Subscriber("mission_state", String, self.on_mission_state)
        # Initialize Publishers:
        self.comms_cycle_status = rospy.Publisher("comms_cycle_status", CommsCycleStatus, queue_size=1)
        self.cycle_graph_pub = rospy.Publisher("cycle_graph", CycleGraph, queue_size=1)
        # Initialize the modem addresses and cycle targets
        rospy.loginfo("[%s] Topics ready, initializing comms cycle" % rospy.Time.now())

    # Setup function
    def configure_comms_cycle(self):
        """This function configures the comms cycle for the vehicle.
        """
        # Get the modem addresses and cycle targets
        self.modem_addresses = configure_modem_addresses(self.num_agents, self.num_landmarks, self.local_address)
        self.cycle_target_mapping = configure_cycle_targets(self.modem_addresses)
        # Generate the landmark priors
        for i in range(self.num_landmarks):
            self.inbound_init_priors["L%d" % i] = {
                "key": "L%d" % i,
                "position": np.array(self.landmarks["L%d" % i]),
                "sigmas": np.array([1.7, 1.7, 0.1])
            }
        # Initialize the partial graph data as a dict of False values with the modem addresses as keys
        for key in self.modem_addresses.keys():
            index = chr(ord("A") + key) # Works because only the agents send partial graphs
            self.inbound_partial_graphs[index] = False
        return

    # Message callback
    def on_tdma_status(self, msg):
        """This function updates the modem TDMA status
        Args:
            msg (TdmaStatus): The TDMA status message
        """
        # Get the TDMA status
        num_slots = 2 * self.num_agents
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
            self.cycle_reset = False
        # Pre-empt the end of the cycle to ensure we complete turnover before the comms_cycle_mgr
        elif current_slot == (num_slots -2) and remaining_sec_in_slot < 3 and self.cycle_reset == False:
            self.on_tdma_cycle_reset()
            self.cycle_reset = True
        # If we need to send messages to other modems:
        if self.num_agents > 1 or self.num_landmarks > 0:
        # Load the cycle message into the queue so its ready to go when we are active
            if time_to_next_active < self.msg_preload and not self.loaded_msg:
                if self.tdma_cycle_sequence == 0:
                    rospy.loginfo("[%s] Publishing Init Prior" % rospy.Time.now())
                    self.init_prior_pub.publish(self.staged_init_prior)
                    self.loaded_msg = True
                else:
                    rospy.loginfo("[%s] Publishing Partial Graph" % rospy.Time.now())
                    self.partial_graph_pub.publish(self.staged_partial_graph)
                    self.loaded_msg = True
            # If we are active, we need to execute the ping cycle
            elif we_are_active == True:
                if elapsed_time_in_slot < 1:
                    rospy.loginfo("[%s] TDMA Active Slot Started, %ssec Remaining" % (rospy.Time.now(), remaining_active_sec))
                    self.loaded_msg = False
                elif elapsed_time_in_slot > self.skew_buffer and remaining_sec_in_slot > self.skew_buffer:
                    self.execute_ping_cycle(current_slot, elapsed_time_in_slot)
                else:
                    pass
            else:
                # If we are not active, do nothing
                pass
        else:
            # This is an escape to prevent the ping cycle from executing if we have no other agents or landmarks
            rospy.logwarn("[%s] No other agents or landmarks to ping" % rospy.Time.now())

        # Regardless, update TDMA
        self.tdma_status = msg
        # self.tdma_status.current_slot = current_slot
        # self.tdma_status.we_are_active = we_are_active
        # self.tdma_status.remaining_slot_seconds = remaining_sec_in_slot
        # self.tdma_status.remaining_active_seconds = remaining_active_sec
        # self.tdma_status.time_to_next_active = time_to_next_active
        # self.tdma_status.slot_duration_seconds = slot_duration
        return

    def on_tdma_cycle_reset(self):
        """This function is called shortly prior to the TDMA cycle reset (when slot=0)
        - This is to prevent the comms_cycle_mgr node and factor_graph_mgr_node from executing at the same time
        - This is called after the last ros message is sent, but possibly before the modem finished the last ping
        """
        cycle_complete, should_smooth = False
        # Check if we've completed initialization (we're in the partial graph phase):
        if self.init_complete:
            num_msg_rcvd = len([v for v in self.inbound_partial_graphs.values() if v == True])
            if num_msg_rcvd < 1:
                rospy.logerr("[%s] No partial graphs received, resetting cycle" % rospy.Time.now())
            elif num_msg_rcvd == 1:
                # Log the cycle failure, but smooth so we can try to reset in the next cycle
                should_smooth = True
                rospy.logwarn("[%s] Only one partial graph received, scrubbing cycle" % rospy.Time.now())
            elif (num_msg_rcvd > 1 and num_msg_rcvd < self.num_agents):
                # NOTE: Sync function is robust to partial inputs, and will prune any unconnected data
                self.synchronize_partial_graphs()
                self.build_cycle_graph()
                # NOTE: Do not smooth because it would ruin the graph
            elif num_msg_rcvd == self.num_agents:
                # Log the cycle success and do not smooth
                cycle_complete = True
            else:
                rospy.logerr("[%s] Received more partial graphs than expected (%d), resetting cycle" % (rospy.Time.now(), num_msg_rcvd))
            # Note that we're clearing the graph data, so we can start fresh for the next cycle
            self.cycle_graph_data.clear()

        # If we are still in the init phase, we need to check if we've received all of the initial priors
        else:
            num_msg_rcvd = len([v for v in self.inbound_init_priors.values() if v is not None])
            if num_msg_rcvd < (1 + self.num_landmarks):
                # Publish that the cycle has failed, but since we didn't get a graph, we can't smooth
                rospy.logerr("[%s] No initial priors received, resetting cycle" % rospy.Time.now())
            elif num_msg_rcvd == (1 + self.num_landmarks):
                # Log the cycle failure, but smooth so we can try to reset in the next cycle
                should_smooth = True
                rospy.logwarn("[%s] Only our initial prior received, scrubbing cycle" % rospy.Time.now())
            elif (num_msg_rcvd > 1 and num_msg_rcvd < (self.num_agents + self.num_landmarks)):
                should_smooth = True
                rospy.logwarn("[%s] Only one initial prior received, scrubbing cycle" % rospy.Time.now())
            elif num_msg_rcvd == (self.num_agents + self.num_landmarks):
                cycle_complete = True
                rospy.loginfo("[%s] All initial priors received, building init prior" % rospy.Time.now())
                self.init_complete = True
            else:
                rospy.logerr("[%s] Received more initial priors than expected (%d), resetting cycle" % (rospy.Time.now(), num_msg_rcvd))
        self.tdma_cycle_sequence += 1
        # Build the comms cycle message
        msg = CommsCycleStatus()
        msg.sequence_number = self.tdma_cycle_sequence
        msg.init_complete = self.init_complete
        msg.cycle_complete = cycle_complete
        msg.should_smooth = should_smooth
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

    # Message Processing Functions
    def on_init_prior(self, msg):
        """This function processes initial prior factor messages
        Args:
            msg (PosePriorFactor): The initial prior factor message
        """
        # Unpack the fields
        local_addr = msg.local_addr
        full_index = msg.full_index
        initial_position = msg.initial_position
        initial_orientation = msg.initial_orientation
        initial_sigmas = msg.initial_sigmas
        # Decode the initial prior factor data
        initial_position, initial_orientation, initial_sigmas = self.decode_init_prior_data_from_int(initial_position, initial_orientation, initial_sigmas)
        # Store in the inbound_init_priors dict
        self.inbound_init_priors[chr(ord("A") + local_addr)] = {
            "key": local_addr+str(full_index),
            "initial_position": initial_position,
            "initial_orientation": initial_orientation,
            "initial_sigmas": initial_sigmas
        }
        rospy.loginfo("[%s] Received Initial Prior Factor from %s" % (rospy.Time.now(),local_addr))
        # Check if all initial prior factors have been received
        self.check_if_all_init_priors_received()
        return

    def check_if_all_init_priors_received(self):
        """This function checks if all initial prior factors have been received
        """
        #Check if the number of keys in the dict is equal to the number of agents
        if len(self.inbound_init_priors) == self.num_agents:
            # If so, we have received all initial prior factors
            rospy.loginfo("[%s] Received all Initial Prior Factors from all agents" % rospy.Time.now())
            # Add the pose priors to a cycle graph message and send to the estimator
            msg = CycleGraph()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            for key, data in self.inbound_init_priors.items():
                pose_prior = PoseWithAssoc()
                pose_prior.key1 = data["key"]
                pose_prior.key2 = data["key"]
                pose_prior.pose.position = Point(*data["initial_position"])
                pose_prior.pose.orientation = Quaternion(*data["initial_orientation"])
                pose_prior.covariance = np.diag(data["initial_sigmas"]**2).flatten().tolist()
                msg.relative_poses.append(pose_prior)
            # Publish the cycle graph message
            self.cycle_graph_pub.publish(msg)
            rospy.loginfo("[%s] Published Cycle Graph with Initial Prior Factors" % rospy.Time.now())
        else:
            rospy.loginfo("[%s] Waiting for all Initial Prior Factors from agents" % rospy.Time.now())
        return

    def on_partial_graph(self, msg):
        """This function processes partial graph messages
        Args:
            msg (PartialGraph): The partial graph message
        """
        # Unpack the fields
        local_addr = chr(msg.local_addr + ord("A"))
        full_index = msg.full_index
        num_poses = msg.num_poses
        # Log that we've recieved the partial graph
        if self.inbound_partial_graphs[local_addr] is False:
            self.inbound_partial_graphs[local_addr] = True
            self.process_inbound_partial_graph(msg) # adds the data to the cycle graph data
            rospy.loginfo("[%s] Received Partial Graph from %s" % (rospy.Time.now(), local_addr))
            # Check if all partial graphs have been received
            self.check_if_all_partial_graphs_received()
        else:
            rospy.logerr("[%s] Received duplicate Partial Graph from %s" % (rospy.Time.now(), local_addr))
        return

    def process_inbound_partial_graph(self, msg:PartialGraph):
        # Unpack the fields
        local_addr = chr(msg.local_addr + ord("A"))
        full_index = msg.full_index
        num_poses = msg.num_poses
        # Generate a list of the expected poses in the partial graph
        expected_poses = [local_addr + str(full_index + i) for i in range(num_poses)]
        # For each relative pose, decode the data:
        for i in range(num_poses):
            position, rotation, sigmas = decode_partial_graph_pose_from_int(
                getattr(msg, f'relative_pos_{i}'),
                getattr(msg, f'relative_rot_{i}'),
                getattr(msg, f'unique_sigmas_{i}')
            )
            # Add to the cycle graph data
            graph_id = f'BTWN_{expected_poses[i]}_{expected_poses[i+1]}'
            self.cycle_graph_data[graph_id] = {
                "key1": expected_poses[i],
                "key2": expected_poses[i+1],
                "position": position,
                "orientation": rotation,
                "sigmas": sigmas
            }
            # For each range measurement, decode the data:
            if i < 4:
                local_symbol = expected_poses[int(getattr(msg, f'local_index_{i}'))]
                remote_addr = chr(getattr(msg, f'remote_addr_{i}')+ord("A"))
                meas_range = decode_partial_graph_range_from_int(getattr(msg, f'meas_range_{i}'))
                # NOTE: by reversing the order of the keys, we match the convention of the existing range entry
                # It will not overwrite the existing entry, because key1 is just a chr but the key1 on file is a symbol
                graph_id = f'RNG_{remote_addr}_{local_symbol}'
                self.cycle_graph_data[graph_id] = {
                    "key1": remote_addr,
                    "key2": local_symbol,
                    "range": meas_range
                }
            elif i < 6:
                local_symbol = expected_poses[int(getattr(msg, f'local_index_{i}'))]
                remote_addr = chr(getattr(msg, f'remote_addr_{i}')+ord("A"))
                remote_index = getattr(msg, f'remote_index_{i}')
                remote_symbol = chr(ord("A") + remote_addr) + str(remote_index)
                # NOTE: by reversing the order of the keys, we match the order of the existing range entry
                # It will not overwrite the existing entry, because key2 is a symb, but key2 on file is a chr
                graph_id = f'RNG_{remote_symbol}_{local_symbol}'
                self.cycle_graph_data[graph_id] = {
                    "key1": remote_symbol,
                    "key2": local_symbol,
                    "range": None
                }
            else:
                rospy.logerr("[%s] Invalid range measurement index: %d" % (rospy.Time.now(), i))
                continue
        return

    def check_if_all_partial_graphs_received(self):
        if len([v for v in self.inbound_partial_graphs.values() if v == True]) == self.num_agents:
            rospy.loginfo("[%s] Received Partial Graphs from all agents" % rospy.Time.now())
            # Synchronize the data
            self.synchronize_partial_graphs()
            # Send the graph and send to the estimator
            self.build_cycle_graph()
        else:
            rospy.loginfo("[%s] Waiting for all Partial Graphs from agents" % rospy.Time.now())
        return

    def synchronize_partial_graphs(self):
        """This function synchronizes the partial graphs once all expected agents have reported
        - It assumes that process_partial_graph has been called for each agent's partial graph
        - It associates the ranges between graphs, checks connectivity and adds range sigmas
        - It can be called on an insufficient number of agents, but will only process the data that is available
        and will remove any unconnected entries
        """
        # Clean up the range associations by associating the duplicate entries and removing the duplicates
        for key in self.cycle_graph_data.keys():
            if key.startswith("RNG") and len(self.cycle_graph_data[key]["key2"]) == 1:
                # Search the remaining RNG entries for a matching key1
                chr = self.cycle_graph_data[key]["key2"]
                for match in self.cycle_graph_data.keys():
                    if match.startswith("RNG") and self.cycle_graph_data[match]["key1"] == self.cycle_graph_data[key]["key1"]:
                        # Double check that the chr matches the key2 of the match
                        if self.cycle_graph_data[match]["key2"][0] == chr:
                            # If so, set the range of the key entry to the value measured in the match
                            self.cycle_graph_data[match]["range"] = self.cycle_graph_data[key]["range"]
                            # Remove the key entry from the cycle graph data
                            del self.cycle_graph_data[key]
                            break
                        else:
                            rospy.logerr("[%s] Mismatched key2 character for range entry %s" % (rospy.Time.now(), key))
                            continue
                    else:
                        rospy.logerr("[%s] No matching association found for range entry %s" % (rospy.Time.now(), key))

        # Now we need to check the connectivity of the cycle graph data
        # Build a list of the keys in each vehicle's pose chain BTWN_key1_key2 -> [key1, key2, ...]
        nodes = []
        for key in self.cycle_graph_data.keys():
            if key.startswith("BTWN"):
                key1 = self.cycle_graph_data[key]["key1"]
                key2 = self.cycle_graph_data[key]["key2"]
                if key1 not in nodes:
                    nodes.append(key1)
                if key2 not in nodes:
                    nodes.append(key2)
        # Then append the landmarks to the poses list
        if self.num_landmarks > 0:
            for i in range(self.num_landmarks):
                nodes.append("L%d" % i)

        # Check that the ranges are associated with a pose and contain a measurement
        for key in self.cycle_graph_data.keys():
            if key.startswith("RNG"):
                key1_check = True
                key2_check = True
                range_check = True
                # Check that the key1 is in the poses list
                if self.cycle_graph_data[key]["key1"] not in nodes:
                    key1_check = False
                    continue
                    # Check that the key2 is in the poses list
                if self.cycle_graph_data[key]["key2"] not in nodes:
                    key2_check = False
                    continue
                # Check that the range is not None
                if self.cycle_graph_data[key]["range"] is None:
                    range_check = False
                    continue

                # If all checks pass, add the range to the cycle graph data
                if key1_check and key2_check and range_check:
                    self.cycle_graph_data[key]["range_sigma"] = self.sigma_range
                elif (not key1_check or not key2_check) and range_check:
                    rospy.logerr("[%s] Recieved range failed to associate, removing" % (rospy.Time.now(), key))
                    del self.cycle_graph_data[key]
                elif (key1_check and key2_check) and not range_check:
                    rospy.logerr("[%s] Transmitted range failed to associate, removing" % (rospy.Time.now(), key))
                    del self.cycle_graph_data[key]
                elif not key1_check and not key2_check :
                    rospy.loginfo("[%s] Range entry %s has no associated in poses, removing" % (rospy.Time.now(), key))
                    del self.cycle_graph_data[key]
                else:
                    rospy.logerr("[%s] Range entry %s is incomplete!" % (rospy.Time.now(), key))

        return

    def build_cycle_graph(self):
        """This function builds the cycle graph from the staged data
        Returns:
            PartialGraph: The built partial graph message
        """
        # Crete the message
        msg = CycleGraph()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # Load all the relative poses from the self.cycle_graph_data dict
        for key, data in self.cycle_graph_data.items():
            if key.startswith("BTWN"):
                relative_pose = PoseWithAssoc()
                relative_pose.key1 = data["key1"]
                relative_pose.key2 = data["key2"]
                relative_pose.pose.position = Point(*data["position"])
                relative_pose.pose.orientation = Quaternion(*data["orientation"])
                relative_pose.covariance = np.diag(data["sigmas"]**2).flatten().tolist()  # Flatten the covariance matrix to a list
                msg.relative_poses.append(relative_pose)
            elif key.startswith("RNG"):
                range_measurments = RangeWithAssoc()
                range_measurments.key1 = data["key1"]
                range_measurments.key2 = data["key2"]
                range_measurments.meas_range = data["range"]
                range_measurments.range_sigma = self.sigma_range
                msg.range_measurements.append(range_measurments)
            else:
                rospy.logerr("[%s] Invalid graph ID: %s" % (rospy.Time.now(), key))
                continue
        # Publish the cycle graph message
        self.cycle_graph_pub.publish(msg)
        rospy.loginfo("[%s] Published Cycle Graph" % rospy.Time.now())

#TODO: Add marginialization for partial graphs from failed cycles.
# Its better to do it here because then you only need to track one partial graph in the comms cycle manager node.
if __name__ == "__main__":

    try:
        graph_mgr = GraphManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[%s] Graph Mgr Interrupted" % rospy.Time.now())
    except Exception as e:
        rospy.logerr("[%s] Graph Mgr Error: %s" % (rospy.Time.now(), e))
    finally:
        rospy.loginfo("[%s] Graph Mgr Exiting" % rospy.Time.now())
        rospy.signal_shutdown("Graph Mgr Exiting")
        exit(0)