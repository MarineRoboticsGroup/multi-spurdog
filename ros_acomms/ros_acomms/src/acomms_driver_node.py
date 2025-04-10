#!/usr/bin/env python3
"""
src.acomms_driver_node
----------------------

This module contains the AcommsDriverNode class, which is handle and publish modem operations.
"""
from __future__ import unicode_literals

import logging
from queue import Queue, Empty
from threading import Thread
from time import sleep, time
from typing import List, Union

from acomms.unifiedlog import UnifiedLog
from acomms.messageparams import (
    Ack,
    Casst,
    CdrAck,
    NavPingDigitalTransponder,
    NavPingGenericTransponder,
    NavStatistics,
    NavTurnTimes,
)
from acomms.micromodem import Micromodem, Message
from acomms.modem_connections import UdpConnection
from acomms.flexibledataprotocol import FDPacket
import rospy

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from ros_acomms_msgs.msg import CST, NST, PDT, PGT, SST, TTA, XST
from ros_acomms_msgs.msg import (
    ModemAck,
    Packet,
    PingReply,
    PingTranspondersReply,
    ReceivedPacket,
)
from ros_acomms_msgs.srv import QueryModemParam, QueryModemParamRequest, QueryModemParamResponse
from ros_acomms_msgs.srv import PingModem, PingModemResponse
from ros_acomms_msgs.srv import PingTransponders, PingTranspondersResponse
from ros_acomms_msgs.srv import QueueTxPacket, QueueTxPacketResponse, GetNextPacketData
from ros_acomms_msgs.srv import QueryModemSrcRequest, QueryModemSrcResponse, QueryModemSrc
from ros_acomms.cfg import acomms_driverConfig
from std_msgs.msg import Header, String, Time
from version_node import version_node

# Import NodeStatus if it is available
try:
    from node_status.msg import NodeStatus
    nodestatusavail = True
except ImportError:
    rospy.logwarn("NodeStatus package not available, can't publish node status.")
    nodestatusavail = False

from datetime import datetime, timezone
from collections import deque, OrderedDict


def convert_datetime_to_rospy(time_stamp):
    # fix the time type
    seconds = time_stamp.replace(tzinfo=timezone.utc).timestamp()
    return rospy.Time.from_sec(seconds)


class AcommsDriverNode(object):
    """
    node for acomms driver
    """

    def __init__(self):
        rospy.init_node("acomms_driver")

        try:
            version = version_node()
            version.getPyAcomms()
            version.getAcomms()
        except:
            rospy.logwarn("Unable to query version information")

        self.carev_checkin: rospy.Time = None

        self.maxsize = 10
        self.ack_publisher = rospy.Publisher('ack', ModemAck, queue_size=self.maxsize)
        self.carev_publisher = rospy.Publisher('carev', Time, queue_size=self.maxsize)
        self.cst_publisher = rospy.Publisher('cst', CST, queue_size=self.maxsize)
        self.nst_publisher = rospy.Publisher('nst', NST, queue_size=self.maxsize)
        self.pdt_publisher = rospy.Publisher('pdt', PDT, queue_size=self.maxsize)
        self.pgt_publisher = rospy.Publisher('pgt', PGT, queue_size=self.maxsize)
        self.sst_publisher = rospy.Publisher('sst', SST, queue_size=self.maxsize)
        self.tta_publisher = rospy.Publisher('tta', TTA, queue_size=self.maxsize)
        self.xst_publisher = rospy.Publisher('xst', XST, queue_size=self.maxsize)

        packet_rx_topic = rospy.names.canonicalize_name(rospy.get_param('~packet_rx_topic', 'packet_rx'))
        self.packet_rx_publisher = rospy.Publisher(
            packet_rx_topic, ReceivedPacket, queue_size=self.maxsize
        )
        self.nmea_publisher = rospy.Publisher("nmea_from_modem", String, queue_size=self.maxsize)
        self.ping_reply_publisher = rospy.Publisher(
            "ping_reply", PingReply, queue_size=self.maxsize
        )
        self.ping_transponders_reply_publisher = rospy.Publisher(
            "ping_transponders_reply", PingTranspondersReply, queue_size=self.maxsize
        )
        self.txf_publisher = rospy.Publisher("txf", Header, queue_size=self.maxsize)
        if nodestatusavail:
            self.pub_nodestatus = rospy.Publisher(
                rospy.names.canonicalize_name("/node_status/" + rospy.get_name()),
                NodeStatus,
                queue_size=self.maxsize,
                latch=True,
            )

        self.publish_nodestatus(False, "Initializing acomms_driver_node...")
        modem_connection_type = rospy.get_param("~modem_connection_type", "serial")
        modem_remote_host = rospy.get_param("~modem_remote_host", "localhost")
        modem_remote_port = rospy.get_param("~modem_remote_port", 4001)
        modem_local_host = rospy.get_param("~modem_local_host", "")
        modem_local_port = rospy.get_param("~modem_local_port", None)
        modem_serial_port = rospy.get_param("~modem_serial_port", "/dev/ttyS3")
        modem_baud_rate = rospy.get_param("~modem_baud_rate", 19200)
        set_modem_time = rospy.get_param("~set_modem_time", False)
        self.modem_config = rospy.get_param("~modem_config", {})

        # pull modem_config params used in dynamic_reconfigure (if there, otherwise use default and add to modem_config dict)
        self.pwramp_txlevel = self.modem_config.get('pwramp.txlevel', None)
        self.tx_inhibit = self.modem_config.get('xmit.txinhibit', None)

        if self.pwramp_txlevel is None:
            self.pwramp_txlevel = rospy.get_param("~pwramp_txlevel", 3) # default is lowest volume (checks top level param)
            self.modem_config['pwramp.txlevel'] = self.pwramp_txlevel

        if self.tx_inhibit is None:
            self.tx_inhibit = False # default is false
            self.modem_config['xmit.txinhibit'] = 1 # when 1:False, 2:True. 0:floating

        # set the updated modem_config dict and pwramp_txlevel on the param server
        rospy.set_param("~modem_config", self.modem_config)
        rospy.set_param("~pwramp_txlevel", self.pwramp_txlevel)

        self.publish_partial_packets = rospy.get_param("~publish_partial_packets", True)

        port = modem_serial_port
        baud = modem_baud_rate
        log_location = "acomms_logs/"
        self.default_modem_dest = rospy.get_param("~default_modem_dest", 121)

        self.use_legacy_packets = rospy.get_param("~use_legacy_packets", False)
        self.use_janus_packets = rospy.get_param("~use_janus_packets", False)
        self.use_tdp = rospy.get_param("~use_tdp", False)

        rospy.loginfo(f'Params\n{rospy.get_param("~")}')
        self.pending_packets = deque()

        # Figure out current log level of this rosnode to pass to pyacomms
        logger = logging.getLogger("rosout")
        level = logger.getEffectiveLevel()

        # First, connect
        self.unified_log = UnifiedLog(
            log_path=log_location,
            console_log_level=level,
            rootname="acomms_logger",
        )
        self.um = Micromodem(
            name="".join([x if x.isalnum() else "_" for x in rospy.get_name()]),
            unified_log=self.unified_log,
            log_level='INFO' if level < logging.DEBUG else 'DEBUG'
        )
        if modem_connection_type == "serial":
            self.um.connect_serial(port, baud)
        elif modem_connection_type == "udp":
            self.um.connection = UdpConnection(
                self.um,
                remote_host=modem_remote_host,
                local_host=modem_local_host,
                remote_port=modem_remote_port,
                local_port=modem_local_port,
            )

        rospy.loginfo(
            "Modem connection open: {}, querying SRC...".format(self.um.connection)
        )

        # Wait here until we can talk to the modem
        connected_to_modem = False
        while not rospy.is_shutdown():
            # This returns None if we time out.  Repeat until we get a reply.
            sleep(0.1)
            if self.um.get_config("SRC", response_timeout=1):
                connected_to_modem = True
                break

        if not connected_to_modem:
            rospy.logfatal(f'Not connected to modem and ros master is shutting down...')
            return
            
        rospy.loginfo("Got modem SRC: {}".format(self.um.id))
        rospy.set_param("modem_src", int(self.um.id))  # make relative to namespace, for other nodes to use

        try:
            self.tat_ms = float(self.um.get_config("TAT", response_timeout=2)["TAT"])
        except TypeError as e:
            rospy.logwarn("Unable to get modem TAT.  Setting to 0, which probably won't work")
            self.tat_ms = 0

        # Hack to work around 7-bit/8-bit addressing problems with legacy packets.
        self.um._ignore_cacyc_address_mismatch = True

        if set_modem_time:
            self.um.set_time()

        for p, v in self.modem_config.items():
            self.um.set_config(p, v)

        # Request all the parameters so they're saved in the log file.
        self.um.get_config_param("TOP")

        # Set up the modem's listeners
        self.um.ack_listeners.append(self.on_modem_msg)
        self.um.carev_listeners.append(self.on_modem_msg)
        self.um.casst_listeners.append(self.on_modem_msg)
        self.um.snnst_listeners.append(self.on_modem_msg)
        self.um.snpdt_listeners.append(self.on_modem_msg)
        self.um.snpgt_listeners.append(self.on_modem_msg)
        self.um.sntta_listeners.append(self.on_modem_msg)

        self.modem_rx_thread = Thread(target=self.modem_rx_handler, daemon=True)
        self.modem_rx_thread.start()

        self.modem_cst_thread = Thread(target=self.modem_cst_handler, daemon=True)
        self.modem_cst_thread.start()

        self.modem_xst_thread = Thread(target=self.modem_xst_handler, daemon=True)
        self.modem_xst_thread.start()

        self.modem_nmea_thread = Thread(target=self.modem_nmea_handler, daemon=True)
        self.modem_nmea_thread.start()

        rospy.Subscriber("nmea_to_modem", String, self.on_nmea_subscriber)

        self.ping_modem_service = rospy.Service(
            "ping_modem", PingModem, self.handle_ping_modem
        )
        self.ping_transponders_service = rospy.Service(
            "ping_transponders", PingTransponders, self.handle_ping_transponders
        )
        self.queue_tx_packet_service = rospy.Service(
            "queue_tx_packet", QueueTxPacket, self.handle_queue_tx_packet
        )

        # Can be used to get the SRC address of the modem, and also serves as a mechanism to block until the acomms
        # driver is up
        self.query_modem_src_service = rospy.Service(
            "query_modem_src", QueryModemSrc, self.handle_query_modem_src
        )

        self.query_modem_param_service = rospy.Service(
            "query_modem_param", QueryModemParam, self.handle_query_modem_param
        )

        # check if we should use the ping payload for message transport
        self.use_ping_payload = rospy.get_param("~use_ping_payload", False)
        # the default is the max bytes we can fit in a miniframe
        self.ping_maximum_miniframe_bytes = rospy.get_param("~ping_maximum_miniframe_bytes", 32)

        # if using the ping payload (or want to use later), configure the service proxy to message_queue
        rospy.loginfo("acomms_driver_node creating get_next_packet_data service proxy")
        self.get_next_packet_data = rospy.ServiceProxy('get_next_packet_data', GetNextPacketData)

        # set flag for first callback only. This syncs dynamic reconf with params passed at launch
        self.first_dynamic_reconf = True
        self.reconfigure_server = DynamicReconfigureServer(
            acomms_driverConfig, self.reconfigure
        )

        self.publish_nodestatus(True, "acomms_driver_node initialized.")

        rospy.loginfo("Acomms driver node initialized.")

    def reconfigure(self, config, level):
        if self.first_dynamic_reconf:
            self.first_dynamic_reconf = False
            # due to way rosparam of typle list doesn't properly map to param server in launch file
            # ..meaning the defaults set in the .cfg file will always use those values at first call
            # ..so to avoid having defaults set (when ros handles this by calling all non-list type ros params from param server),
            # ..set values from init in config
            config["tx_inhibit"] = self.tx_inhibit
            config["pwramp_txlevel"] = self.pwramp_txlevel

            config["use_ping_payload"] = self.use_ping_payload
            config["ping_maximum_miniframe_bytes"] = self.ping_maximum_miniframe_bytes

            rospy.logdebug(f'DEBUG: First dynamic_reconfigure call, syncing config from init')
            return config
            
        rospy.loginfo("Acomms driver reconfigure request received.")

        self.pwramp_txlevel = config["pwramp_txlevel"]
        self.tx_inhibit = config["tx_inhibit"]
            
        self.use_ping_payload = config["use_ping_payload"]
        self.ping_maximum_miniframe_bytes = config["ping_maximum_miniframe_bytes"] # dynamic reconf has a min:0 max:32
    
        # we always set all the params passed rather than checking state. 
        # .. e.g., If the modem is booted into another slot etc.. we may need to set the same value again
        rospy.loginfo(f"Acomms driver reconfigure request received. config={config}")

        um_response = self.um.set_config("xmit.txinhibit", 2 if self.tx_inhibit else 1, response_timeout=5)
        rospy.loginfo(f"Acomms driver reconfigure xmit.txinhibit um_response: {um_response}")

        um_response = self.um.set_config("pwramp.txlevel", self.pwramp_txlevel, response_timeout=5)
        rospy.loginfo(f"Acomms driver reconfigure pwramp.txlevel um_response: {um_response}")

        return config

    def verify_modem_connection(self, missed_checkin_factor: int = 5):
        secs_since_checkin = rospy.get_rostime().secs - self.carev_checkin.secs
        cycle_timeout_factor = self.um.config_data['CTO'] * missed_checkin_factor
        return True if secs_since_checkin < cycle_timeout_factor else False

    def handle_query_modem_src(self, query: QueryModemSrcRequest) -> QueryModemSrcResponse:
        response = QueryModemSrcResponse(src=self.um.id)
        return response

    def on_modem_msg(self, parsed_obj: Union[Message, Ack, Casst, NavStatistics, NavTurnTimes,
                                             NavPingDigitalTransponder, NavPingGenericTransponder],
                     msg: Message = None) -> None:
        self.um._daemon_log.debug(f'/on_modem_msg.MSG: {str(msg).strip()}')
        self.um._daemon_log.debug(f'/on_modem_msg.{type(parsed_obj).__name__}: {parsed_obj}')

        # only case: acomms.MessageParser.CAREV modem callback
        if msg and isinstance(parsed_obj, Message) and 'CAREV' in msg:
            # useful to check/verify connection to uModem
            self.carev_checkin = rospy.get_rostime()
            self.carev_publisher.publish(Time(data=self.carev_checkin))

        elif isinstance(parsed_obj, Ack):
            self.publish_ack(parsed_obj)
        elif isinstance(parsed_obj, Casst):
            self.publish_sst(parsed_obj)
        elif isinstance(parsed_obj, NavStatistics):
            self.publish_nst(parsed_obj)
        elif isinstance(parsed_obj, NavPingDigitalTransponder):
            self.publish_pdt(parsed_obj)
        elif isinstance(parsed_obj, NavPingGenericTransponder):
            self.publish_pgt(parsed_obj)
        elif isinstance(parsed_obj, NavTurnTimes):
            self.publish_tta(parsed_obj)

    def publish_nodestatus(self, bool_ok, fault_detail=None):
        """method to publish the current node status if it is available

        Args:
            bool_ok (_type_): whether or not the node is initialized
            fault_detail (_type_, optional): _description_. Defaults to None.
        """

        if nodestatusavail:
            msg = NodeStatus()
            msg.header.stamp = rospy.Time.now()
            msg.ok = bool_ok
            if fault_detail is None:
                if msg.ok:
                    fault_detail = ""
                else:
                    fault_detail = "Error"

            msg.fault_detail = fault_detail
            self.pub_nodestatus.publish(msg)

    def modem_rx_handler(self):
        """method to handle modem rx"""
        incoming_all_packet_queue = Queue()
        self.um.incoming_all_packet_queues.append(incoming_all_packet_queue)
        while not rospy.is_shutdown():
            try:
                incoming_packet: FDPacket = incoming_all_packet_queue.get(block=True)
                # publish complete (no bad CRC) packets
                if not incoming_packet.packet_is_good:
                    rospy.loginfo("Got bad packet (at least one bad frame)")
                    # If we aren't publishing partial packets, skip this one
                    if not self.publish_partial_packets:
                        continue
                    # Otherwise, clear all frames after the first bad frame and publish it
                    has_gone_bad = False
                    for miniframe in incoming_packet.miniframes:
                        if not miniframe.crccheck or has_gone_bad:
                            has_gone_bad = True
                            miniframe.data = bytearray()
                    for dataframe in incoming_packet.dataframes:
                        if not dataframe.crccheck or has_gone_bad:
                            has_gone_bad = True
                            dataframe.data = bytearray()

                packet_msg = Packet(
                    src=incoming_packet.src,
                    dest=incoming_packet.dest,
                    miniframe_rate=incoming_packet.miniframe_rate,
                    dataframe_rate=incoming_packet.dataframe_rate,
                    miniframe_bytes=incoming_packet.minibytes,
                    dataframe_bytes=incoming_packet.databytes,
                )
                cst_values = incoming_packet.cyclestats.copy()
                # Fix the time type
                cst_values["toa"] = convert_datetime_to_rospy(cst_values["toa"])
                cst_msg = CST(**cst_values)

                hdr = Header(stamp=rospy.get_rostime())
                valid_miniframe_bytes = incoming_packet.valid_minibytes.flat
                valid_dataframe_bytes = incoming_packet.valid_databytes.flat

                rospy.logwarn("valid_minibytes: {}".format(valid_miniframe_bytes))
                rospy.logwarn("valid_databytes: {}".format(valid_dataframe_bytes))
                rx_packet_msg = ReceivedPacket(
                    header=hdr,
                    packet=packet_msg,
                    minibytes_valid=valid_miniframe_bytes,
                    databytes_valid=valid_dataframe_bytes,
                    cst=cst_msg,
                )
                self.packet_rx_publisher.publish(rx_packet_msg)

            except Exception as e:
                rospy.logerr_throttle(1, "Error in packet RX handler: {}".format(e))

    def modem_cst_handler(self):
        """method to handle incoming cst messages"""
        incoming_cst_queue = Queue()
        self.um.attach_incoming_cst_queue(incoming_cst_queue)

        while not rospy.is_shutdown():
            try:
                incoming_cst = incoming_cst_queue.get(block=True)
                self.publish_cst(incoming_cst)
            except Exception as e:
                rospy.logerr_throttle(1, "Error in CST handler: {}".format(e))

    def modem_xst_handler(self):
        """method to handle incoming xst messages"""
        incoming_xst_queue = Queue()
        self.um.attach_incoming_xst_queue(incoming_xst_queue)

        while not rospy.is_shutdown():
            try:
                incoming_xst = incoming_xst_queue.get(block=True)
                self.publish_xst(incoming_xst)
            except Exception as e:
                rospy.logerr_throttle(1, "Error in XST handler: {}".format(e))

    def modem_nmea_handler(self):
        """method to handle incoming nmea messages"""
        incoming_nmea_queue = Queue()
        self.um.attach_incoming_msg_queue(incoming_nmea_queue)

        while not rospy.is_shutdown():
            try:
                incoming_string = incoming_nmea_queue.get(block=True)
                if not isinstance(incoming_string, str):
                    incoming_string = incoming_string["raw"]
                # publish it
                nmea_msg = String(data=incoming_string)
                self.nmea_publisher.publish(nmea_msg)

                # TODO: move most of this functionality into pyacomms
                # Watch here for a handful of relevant messages.
                if "CATXF" in incoming_string:
                    txf_msg = Header(stamp=rospy.get_rostime())
                    self.txf_publisher.publish(txf_msg)

                # if any params we set with dynamic reconfigure flow through, update our value
                # .. so future calls to dynamic reconfigure set the latest value
                if "CACFG" in incoming_string:
                    self.handle_config_update(cacfg_nmea=incoming_string)

            except Exception as e:
                rospy.logerr_throttle(1, "Error in NMEA RX handler: {}".format(e))

    def handle_config_update(self, cacfg_nmea):
        # any params added to dynamic reconfigure should add a conditional to this method
        # this methods intent is to capture those values as they are read in a $CACFG
        is_dynamic_reconf_param = False
        try:
            value = cacfg_nmea.split('*')[0].split(',')[-1]
            if 'pwramp.txlevel' in cacfg_nmea:
                is_dynamic_reconf_param = True
                self.pwramp_txlevel = int(value)
                
            elif 'xmit.txinhibit' in cacfg_nmea:
                is_dynamic_reconf_param = True
                if int(value) == 2:
                    self.tx_inhibit = True
                else:
                    self.tx_inhibit = False
        except ValueError:
            rospy.logwarn(f'WARNING: ValueError when updating modem config value managed by dynamic reconfigure! modem msg: {cacfg_nmea}')
            rospy.logwarn(f'WARNING: Leaving config value as-is. A call to dynamic reconfigure will use the old value.')
        else:
            if is_dynamic_reconf_param: rospy.logdebug(f'DEBUG: Got a CACFG for a param used in dynamic_reconfigure! Updating: {cacfg_nmea}')

    def handle_ping_transponders(self, request):
        """handler for rospy PingTransponders service class

        Args:
            request (_type_): info about the transponder

        Returns:
            (PingTranspondersResponse): service call response from the transponder
        """
        rospy.loginfo("Requesting modem send transponder ping")

        # request.transponder_dest_mask = request.transponder_dest_mask.decode('utf-8')

        params = [
            str(1),
            str(1),
            str(0),
            str(0),
            str(request.timeout_sec * 1000),
            str(1 if request.transponder_dest_mask[0] else 0),
            str(1 if request.transponder_dest_mask[1] else 0),
            str(1 if request.transponder_dest_mask[2] else 0),
            str(1 if request.transponder_dest_mask[3] else 0),
        ]
        msg = {"type": "CCPDT", "params": params}

        incoming_msg_queue = Queue()
        self.um.attach_incoming_msg_queue(incoming_msg_queue)
        rospy.sleep(0.2)
        self.um.write_nmea(msg)

        matching_msg = None
        # add 1 second to ros driver timeout to handle race condition
        # .. e.g., one of the transponders requested did not reply so the modem waits the full timeout 
        # .. before printing the SNTTA message
        remaining_time = request.timeout_sec + 1
        end_time = time() + remaining_time

        while remaining_time > 0:
            try:
                new_msg = incoming_msg_queue.get(timeout=remaining_time)
                if new_msg["type"] in "SNTTA":
                    matching_msg = new_msg
                    break
                else:
                    remaining_time = end_time - time()
                    continue
            except Empty:
                break
        sntta = matching_msg

        self.um.detach_incoming_msg_queue(incoming_msg_queue)

        if sntta:
            rospy.loginfo(f"Response from transponders: {sntta}")
            time_of_ping = datetime.strptime(
                datetime.now().strftime("%Y%m%d") + sntta["params"][4],
                "%Y%m%d%H%M%S.%f",
            )
            msg = PingTranspondersReply(
                transponder_dest_mask=request.transponder_dest_mask,  # .encode('utf-8'),
                timeout_sec=request.timeout_sec,
                owtt_a=float(sntta["params"][0]) if sntta["params"][0] != "" else -1.0,
                owtt_b=float(sntta["params"][1]) if sntta["params"][1] != "" else -1.0,
                owtt_c=float(sntta["params"][2]) if sntta["params"][2] != "" else -1.0,
                owtt_d=float(sntta["params"][3]) if sntta["params"][3] != "" else -1.0,
                time_of_ping=convert_datetime_to_rospy(time_of_ping),
            )
            msg.header.stamp = convert_datetime_to_rospy(time_of_ping)
            self.ping_transponders_reply_publisher.publish(msg)

            response = PingTranspondersResponse(
                travel_times=[
                    float(sntta["params"][0]) if sntta["params"][0] != "" else -1.0,
                    float(sntta["params"][1]) if sntta["params"][1] != "" else -1.0,
                    float(sntta["params"][2]) if sntta["params"][2] != "" else -1.0,
                    float(sntta["params"][3]) if sntta["params"][3] != "" else -1.0,
                ]
            )
        else:
            rospy.loginfo(f"timed-out before SNTTA....")
            response = PingTranspondersResponse(travel_times=[-1.0, -1.0, -1.0, -1.0])

        rospy.loginfo(f"Service call response: {response}")

        return response

    def handle_query_modem_param(self, req: QueryModemParamRequest) -> QueryModemParamResponse:
        """handler for rospy QueryModemParam service class
        Returns:
            QueryModemParamResponse.value <str> : value of the requested parameter
                - value is of type <str> because uModem param values can be combination of
                    - <int>, <str>, <dict> (from Micromodem.get_config call), etc
                    caller must know what type of data to expect in response
        """
        rospy.logdebug("Requesting modem param: {}".format(req.param))
        try:
            param_dict = self.um.get_config(req.param)
            if param_dict is None:
                raise rospy.ServiceException(f"Unable to Execute Service query_modem_param (param returned None)")
        except KeyError as e:
            raise rospy.ServiceException(f"Unable to Execute Service query_modem_param: {e}")
        v = str(param_dict) if len(param_dict) > 1 else str(param_dict[req.param])
        return QueryModemParamResponse(value=v)

    def handle_ping_modem(self, request):
        """handler for rospy PingModem service class

        Args:
            request (_type_): info about the modem

        Returns:
            (PingModemResponse): ping response from the modem
        """
        rospy.loginfo("Requesting modem send ping")
        ping_payload = request.hexdata

        if self.use_ping_payload and self.get_next_packet_data:
            rospy.loginfo(f"Attempting to fill ping payload for ROS msg transport. payload max size: {self.ping_maximum_miniframe_bytes} bytes")
            try:
                packet_data_response = self.get_next_packet_data(num_miniframe_bytes=self.ping_maximum_miniframe_bytes, num_dataframe_bytes=0)
            except:
                rospy.logwarn(f"ERROR: problem with get_next_packet_data() service. Not sending a payload with this ping...")
            else:
                if packet_data_response.num_messages > 0:
                    rospy.loginfo(f"Received data to send with ping request length: {len(packet_data_response.miniframe_bytes)}")
                    ping_payload = packet_data_response.miniframe_bytes

        self.um.send_fdp_ping(request.dest, request.rate, request.cdr, ping_payload)
        if request.timeout_sec < 1:
            request.timeout_sec = 1
        ping_reply, cst = self.um.wait_for_fdp_ping_reply(
            include_cst=True, timeout=request.timeout_sec
        )

        # CCCMD,PNG: SRC=(unit designated as ping originator), DEST=(unit designated as receiver of the ping)
        # CACMA,PNG: SRC=(unit designated as transmitter),     DEST=(unit designated as receiver of the ping)
        # CACMR,PNR: SRC=(unit designated as transmitter),     DEST=(unit designated as receiver of the ping)

        if not ping_reply:
            response = PingModemResponse(timed_out=True)
        elif ping_reply["dest"] != self.um.id:
            response = PingModemResponse(timed_out=True)
        else:
            rospy.loginfo("Received ping response")
            response = PingModemResponse(
                timed_out=False,
                one_way_travel_time=ping_reply["owtt"],
                tat=self.tat_ms / 1000,
                txlevel=ping_reply["tx_level"],
                timestamp_resolution=ping_reply["timestamp_res"],
                toa_mode=ping_reply["toa_mode"],
                snv_on=ping_reply["snv_on"],
                timestamp=ping_reply["timestamp"],
            )

            msg = PingReply(
                src=ping_reply["src"],
                dest=ping_reply["dest"],
                owtt=ping_reply["owtt"],
                tat=self.tat_ms / 1000,
                snr_in=ping_reply["snr_in"],
                snr_out=ping_reply["snr_out"],
                tx_level=ping_reply["tx_level"],
                timestamp_res=ping_reply["timestamp_res"],
                toa_mode=ping_reply["toa_mode"],
                snv_on=ping_reply["snv_on"],
                timestamp=ping_reply["timestamp"],
            )

            if cst is not None:
                cst_values = cst.copy()
                # Fix the time type
                cst_values["toa"] = convert_datetime_to_rospy(cst_values["toa"])
                response.cst = CST(**cst_values)
                msg.cst = CST(**cst_values)

            self.ping_reply_publisher.publish(msg)

        return response

    def on_nmea_subscriber(self, msg):
        """writes nmea message to micromodem

        Args:
            msg (_type_): NMEA message to send, only sends the data field of the message
        """
        self.um.write_nmea(msg.data)

    def handle_queue_tx_packet(self, queue_tx_packet_req):
        """handler for queuing a tx packaet for transmission

        Args:
            queue_tx_packet_req (_type_): info about the modem

        Returns:
            (QueueTxPacketResponse): ping response from the modem
        """
        rospy.loginfo(
            "Queuing new packet for acomms transmission: {}".format(queue_tx_packet_req)
        )
        queue_tx_packet_resp = QueueTxPacketResponse()
        if queue_tx_packet_req.insert_at_head:
            self.pending_packets.append(queue_tx_packet_req.packet)
            queue_tx_packet_resp.position_in_queue = 0
            queue_tx_packet_resp.success = True
        else:
            self.pending_packets.appendleft(queue_tx_packet_req.packet)
            queue_tx_packet_resp.position_in_queue = len(self.pending_packets) - 1
            queue_tx_packet_resp.success = True
        return queue_tx_packet_resp

    def _send_packet(self, packet: Packet):
        rospy.loginfo("Sending message via acomms")

        if self.use_janus_packets:
            rospy.loginfo("Sending JANUS packet: {}".format(packet))
            self.um.send_tjn(
                dest_id=packet.dest,
                ack=0,
                hex_header="",
                hex_data=packet.dataframe_bytes.hex(),
            )
        elif self.use_legacy_packets:
            rospy.loginfo("Sending Legacy Packet: {}".format(packet))
            self.um.send_packet_data(
                packet.dest,
                packet.dataframe_bytes,
                rate_num=packet.dataframe_rate,
                ack=False,
            )
        else:
            ack = 0
            header_data = 0
            rospy.loginfo("Sending FD Packet: {}".format(packet))
            fdpacket = FDPacket(
                packet.src,
                packet.dest,
                miniframe_rate=packet.miniframe_rate,
                dataframe_rate=packet.dataframe_rate,
                ack=ack,
                minibytes=packet.miniframe_bytes,
                databytes=packet.dataframe_bytes,
            )

            try:
                self.um.send_fdpacket(fdpacket, self.use_tdp)
            except:
                self.um.send_fdpacket(fdpacket)

    def process_outgoing_queue(self):
        """tries to sending any pending packets"""
        try:
            if len(self.pending_packets) > 0:
                self._send_packet(self.pending_packets.pop())
        except:
            pass

    # def on_modem_rxframe(self, dataframe):
    #     # Check to see if we care about this message
    #     data_bytes = dataframe.data
    #
    #     try:
    #         if (data_bytes[0] == self.ros_header_byte) and (data_bytes[1] == 0x02):
    #             header_byte, msg_id_byte, header_secs = struct.unpack('>BBI', data_bytes)
    #             self.pending_targets.pop(header_secs, None)
    #             rospy.loginfo("Got ack for {}".format(header_secs))
    #
    #     except Exception as e:
    #         rospy.logwarn("Error parsing acomms frame: {}".format(e))
    #         pass

    def publish_ack(self, ack: Ack) -> None:
        ack_msg = ModemAck(header=Header(stamp=rospy.get_rostime()),
                           src=ack.src,
                           dest=ack.dest)
        if isinstance(ack, CdrAck):
            ack_msg.ack_type = ModemAck.ACK_TYPE_FDP
            ack_msg.rx_sequence_num = ack.rxseqnum
            ack_msg.minirate = ack.minirate
            ack_msg.miniframe_ack = ack.miniframe_ack
            ack_msg.dataframe_ack = ack.dataframe_ack
        else:
            # legacy ACK
            ack_msg.ack_type = ModemAck.ACK_TYPE_LEGACY
            ack_msg.rx_sequence_num = -1
            ack_msg.minirate = -1
            ack_msg.miniframe_ack = 0
            ack_msg.dataframe_ack = 0x01 << ack.frame_num

        self.ack_publisher.publish(ack_msg)

    def publish_cst(self, cst):
        """publishes cst message to the cst_publisher

        Args:
            cst (_type_): _description_
        """
        cst_values = cst.copy()

        # Fix the time type
        cst_values["toa"] = convert_datetime_to_rospy(cst_values["toa"])

        msg = CST(**cst_values)
        self.cst_publisher.publish(msg)

    def publish_pdt(self, pdt: NavPingDigitalTransponder) -> None:
        try:
            msg = PDT(header=Header(stamp=rospy.get_rostime()),
                      grp=pdt.grp,
                      chn=pdt.chn,
                      lf=pdt.lf,
                      nav_agn=pdt.nav_agn,
                      timeout=pdt.timeout,
                      flags=pdt.flags)
        except Exception as e:
            rospy.logerr(f'/publish_pdt.ERROR: {(e,)}')
        else:
            self.pdt_publisher.publish(msg)

    def publish_pgt(self, pgt: NavPingGenericTransponder) -> None:
        try:
            msg = PGT(header=Header(stamp=rospy.get_rostime()),
                      grp=pgt.grp,
                      chn=pgt.chn,
                      lf=pgt.lf,
                      nav_agn=pgt.nav_agn,
                      timeout=pgt.timeout,
                      flags=pgt.flags)
        except Exception as e:
            rospy.logerr(f'/publish_pgt.ERROR: {(e,)}')
        else:
            self.pgt_publisher.publish(msg)

    def publish_nst(self, nst: NavStatistics) -> None:
        # Fix the time type
        nav_query_time = datetime.strptime(nst.query_time, '%Y%m%d%H%M%S.%f')
        try:
            msg = NST(header=Header(stamp=rospy.get_rostime()),
                      version=nst.version,
                      ftx=nst.ftx,
                      ttx=nst.ttx,
                      query_time=convert_datetime_to_rospy(nav_query_time),
                      nav_agn=nst.nav_agn,
                      tat=nst.tat,

                      a1_owtt=nst.xpond_A['owtt'],
                      a1_pk=nst.xpond_A['pk'],
                      a1_pow=nst.xpond_A['pow'],
                      a1_ratio=nst.xpond_A['ratio'],
                      a1_frx=nst.xpond_A['frx'],

                      b2_owtt=nst.xpond_B['owtt'],
                      b2_pk=nst.xpond_B['pk'],
                      b2_pow=nst.xpond_B['pow'],
                      b2_ratio=nst.xpond_B['ratio'],
                      b2_frx=nst.xpond_B['frx'],

                      c3_owtt=nst.xpond_C['owtt'],
                      c3_pk=nst.xpond_C['pk'],
                      c3_pow=nst.xpond_C['pow'],
                      c3_ratio=nst.xpond_C['ratio'],
                      c3_frx=nst.xpond_C['frx'],

                      d4_owtt=nst.xpond_D['owtt'],
                      d4_pk=nst.xpond_D['pk'],
                      d4_pow=nst.xpond_D['pow'],
                      d4_ratio=nst.xpond_D['ratio'],
                      d4_frx=nst.xpond_D['frx'])
        except Exception as e:
            rospy.logerr(f"/publish_nst.ERROR: {(e,)}")
        else:
            self.nst_publisher.publish(msg)

    def publish_tta(self, tta: NavTurnTimes) -> None:
        # TODO: Fix the time type
        # time_of_ping currently in format %H%M%S.%f (<float>
        # converting to datetime sets YMD to 1900, 01, 01
        # keep as float type, for now
        # =======================
        # dt = datetime.strptime(tta_values['time_of_ping'], '%H%M%S.%f')
        # tta.time_of_ping = convert_datetime_to_rospy(dt)
        # =======================
        try:
            msg = TTA(header=Header(stamp=rospy.get_rostime()),
                      time_of_ping=tta.time_of_ping,
                      a_flag=tta.a_flag,
                      b_flag=tta.b_flag,
                      c_flag=tta.c_flag,
                      d_flag=tta.d_flag)
        except Exception as e:
            rospy.logerr(f"/publish_tta.ERROR: {(e,)}")
        else:
            self.tta_publisher.publish(msg)

    def publish_xst(self, xst):
        """publishes xst message to the xst_publisher

        Args:
            xst (_type_): _description_
        """
        xst_values = xst.copy()

        # Fix the time type
        xst_values["time"] = convert_datetime_to_rospy(xst_values["time"])

        msg = XST(**xst_values)
        self.xst_publisher.publish(msg)

    def publish_sst(self, sst: Casst) -> None:
        """publishes sst message to the sst_publisher

        Args:
            sst (_type_): _description_
        """
        msg = SST(
            time=rospy.get_rostime(),
            sst_version=sst.sst_version,
            in_water_spl_dB=sst.in_water_spl_dB,
            detector=sst.detector,
            num_samples=sst.num_samples,
            summary_min=sst.summary["min"],
            summary_lower_quartile=sst.summary["lower_quartile"],
            summary_median=sst.summary["median"],
            summary_upper_quartile=sst.summary["upper_quartile"],
            summary_max=sst.summary["max"],
            summary_len=sst.summary["len"],
        )
        self.sst_publisher.publish(msg)

    def close(self):
        """disconnects from the micromodem"""
        self.um.disconnect()


if __name__ == "__main__":
    node = None
    try:
        node = AcommsDriverNode()

        rospy.loginfo("Acomms node started")

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
            node.process_outgoing_queue()

        node.close()
        rospy.loginfo("Acomms node shutdown")

    except rospy.ROSInterruptException:
        if node is not None:
            node.close()
        rospy.loginfo("Acomms node shutdown (interrupt)")
