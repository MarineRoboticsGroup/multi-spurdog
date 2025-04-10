#!/usr/bin/env python3
import rospy
import roslib.message
from rospy import AnyMsg
from std_msgs.msg import Header, Float32
from ros_acomms_msgs.msg import PingReply, CST, XST, TransmittedPacket, ReceivedPacket, Packet, SST, AcousticRange
from ros_acomms_msgs.srv import PingModem, PingModemResponse, QueueTxPacket, QueueTxPacketResponse, \
    QueryModemSrc, QueryModemSrcRequest, QueryModemSrcResponse, GetNextPacketData
from ros_acomms_modeling.msg import SimPacket
from ros_acomms_modeling.srv import SimTransmissionLoss, SimTransmissionLossRequest, SimPacketPerformance, \
    SimPacketPerformanceRequest, SimPacketPerformanceResponse, SimTravelTime, SimTravelTimeRequest, SimTravelTimeResponse
from queue import Queue, Empty, Full
from threading import Thread, Event
from collections import namedtuple, deque
import datetime
from geopy.distance import distance as geopy_distance
import numpy as np
from typing import Optional, List
# Use ros_groundtruth messages for sim if available, otherwise use internal versions.
try:
    from groundtruth.msg import Tick, Tock
except ImportError:
    from ros_acomms_modeling.msg import Tick, Tock

ActivePacket = namedtuple('ActivePacket', [
                          'sim_packet', 'arrival_time', 'finish_time', 'receive_level_db'])
Position = namedtuple('Position', ['latitude', 'longitude', 'depth'])

def sum_incoherent_db(levels_in_db):
    levels_in_db = np.asarray(levels_in_db)
    return 10.0 * np.log10(np.sum(10**(levels_in_db/10)))

def deserialze_anymsg(msg_data: AnyMsg):
    topic_type = msg_data._connection_header['type']
    topic_class = roslib.message.get_message_class(topic_type)
    msg = topic_class()
    msg.deserialize(msg_data._buff)
    return msg

class PingReplyTransaction(object):
    PING_TYPE_NONE = 0
    PING_TYPE_REQUEST = 1
    PING_TYPE_REPLY = 2
    ''' Constants in SimPacket.msg rel to ping
    no-ping -> PING_TYPE_NONE ----- no action (normal sim_packet)
    start ---> PING_TYPE_REQUEST -- when we are requesting a PingReply from another node
    finish --> PING_TYPE_REPLY ---- when the PingReply is a response to a prior ping request
    '''
    def __init__(self, transaction_start_t, event):
        super(PingReplyTransaction, self).__init__()
        self._cancel_request_active = False
        self.transaction_start_t = transaction_start_t
        self.transaction_end_t = None
        self.sim_packet_reply = None
        self.event = event
        self.timeout = None

    @property
    def cancel_request_active(self):
        return self._cancel_request_active

    @cancel_request_active.setter
    def cancel_request_active(self, value):
        self._cancel_request_active = value

    def complete_transaction(self, sim_packet_reply):
        if sim_packet_reply.ping_transaction_start == self.transaction_start_t:
            # We have a match! We can stop waiting
            self.sim_packet_reply = sim_packet_reply
        else:
            raise KeyError('''Error: transaction start times do not match.
                orig start time:      {}
                bad match start time: {}'''.format(self.transaction_start_t,
                                                   sim_packet_reply.ping_transaction_start))
        self.event.set()
        return self.event

    def timed_out(self):
        self.sim_packet_reply = None
        self.event.set()
        return self.event

    def cancel_transaction(self):
        self.cancel_request_active = True
        if not self.use_sim_tick:
            rospy.sleep(0.05)

    def wait(self, timeout=None):
        if timeout:
            self.timeout = timeout

        # when other thread calls event.set() this will stop waiting
        self.event.wait()
        self.transaction_end_t = rospy.Time.now()

        if self.sim_packet_reply is None:
            raise TimeoutError("Timed out waiting for ping reply!")

        if self.cancel_request_active:
            raise ConnectionAbortedError(
                "Waiting for ping reply aborted by user. Returning...")

        return self.sim_packet_reply

class ModemSimNode(object):
    def __init__(self):
        self.maxsize = 10
        self.packet_transmit_queue = Queue()
        self.packets_in_the_water: List[ActivePacket] = []
        self.active_rx_packets: List[ActivePacket] = []
        self.leftover_rx_packets: List[ActivePacket] = []
        self.incoming_packets_queue = Queue()

        # The sim tick event, used for synchronizing sim tick subscriber and processing loops
        self.sim_tick_queue = Queue(maxsize=1)

        rospy.init_node('modem_sim_node', log_level=rospy.INFO)
        self.node_name = rospy.get_name()

        self.bw = rospy.get_param('~bandwidth_hz', default=5000)
        self.fc = rospy.get_param('~center_frequency_hz', default=10000)
        self.src = rospy.get_param('~SRC', default=0)
        self.verbose = rospy.get_param('~verbose', default=None)
        if self.verbose is None:
            # user didn't set this param. Since it's new and it suppresses logging, print warning
            rospy.logwarn(f'NOTICE: 2024-05-31 New rosparam \"~verbose\" by default is False.')
            rospy.logwarn(f'NOTICE: Setting \"~verbose\" to True means a more verbose logging output.')
            rospy.logwarn(f'NOTICE:  (\"~verbose\"=True will show the log level output prior to this new param)')
            self.verbose = False
        else:
            self.verbose = bool(self.verbose)
        self.use_tick_time = rospy.get_param('/use_tick_time', default=False)
        rospy.loginfo("use_tick_time {} ".format(self.use_tick_time))

        # This will be overridden by a published value, if one appears
        self.ambient_noise = rospy.get_param('~ambient_noise_db', default=60)

        use_sim_tick_name = rospy.search_param('use_sim_tick')
        if use_sim_tick_name:
            self.use_sim_tick = rospy.get_param(
                use_sim_tick_name, default=False)
        else:
            self.use_sim_tick = False
        rospy.loginfo("use_sim_tick {} ".format(self.use_sim_tick))

        tick_topic_name_param = rospy.search_param('tick_topic_name')
        rospy.loginfo("tick_topic_name {} ".format(tick_topic_name_param))
        if tick_topic_name_param:
            self.tick_topic_name = rospy.get_param(
                tick_topic_name_param, default='tick')
        else:
            self.tick_topic_name = 'tick'
        tock_topic_name_param = rospy.search_param('tock_topic_name')
        if tock_topic_name_param:
            self.tock_topic_name = rospy.get_param(
                tock_topic_name_param, default='tock')
        else:
            self.tock_topic_name = 'tock'

        use_sim_time_name = rospy.search_param('use_sim_time')
        if use_sim_time_name:
            self.use_sim_time = rospy.get_param(
                use_sim_time_name, default=False)
        else:
            self.use_sim_time = False
        rospy.loginfo("use_sim_time {} ".format(self.use_sim_time))

        location_source_param = rospy.get_param('~modem_location_source', default=None)
        if location_source_param:
            rospy.logwarn("modem_location_source and the GetPlatformLocation service is no longer supported. " +
                          " Use modem_location_topic to specify a topic" +
                          " that publishes location (defaults to 'location') or use the latitude/longitude/depth" +
                          " params to specify a static location.  This warning will be removed in a future release.")

        modem_location_topic = rospy.get_param('~modem_location_topic', default='location')
        latitude = rospy.get_param('~latitude', default=None)
        longitude = rospy.get_param('~longitude', default=None)
        depth = rospy.get_param('~depth', default=None)
        if not longitude or not latitude or not depth:
            # if we don't have a static position, we need to wait for a position to arrive.
            rospy.loginfo(f"No static position was specified, waiting for modem location on {modem_location_topic}")
            msg = rospy.wait_for_message(modem_location_topic, AnyMsg)
            location = deserialze_anymsg(msg)
            latitude = location.latitude
            longitude = location.longitude
            depth = location.depth

        self.modem_location = Position(latitude, longitude, depth)
        rospy.Subscriber(modem_location_topic, AnyMsg, self.on_modem_location)

        packet_rx_topic = rospy.names.canonicalize_name(rospy.get_param('~packet_rx_topic', 'packet_rx'))
        self.rx_publisher = rospy.Publisher(
            packet_rx_topic, ReceivedPacket, queue_size=self.maxsize)
        self.tx_publisher = rospy.Publisher(
            'transmit', TransmittedPacket, queue_size=self.maxsize)
        self.sim_publisher = rospy.Publisher(
            '/acoustic_channel', SimPacket, queue_size=self.maxsize)
        self.cst_publisher = rospy.Publisher(
            'cst', CST, queue_size=self.maxsize)
        self.xst_publisher = rospy.Publisher(
            'xst', XST, queue_size=self.maxsize)
        self.sst_publisher = rospy.Publisher(
            'sst', SST, queue_size=self.maxsize)
        if self.use_sim_tick:
            self.last_step_id = 0
            self.sim_tick_subscriber = rospy.Subscriber(
                self.tick_topic_name, Tick, self.on_sim_tick)
            self.sim_tock_publisher = rospy.Publisher(
                self.tock_topic_name, Tock, queue_size=10, latch=True)
            rospy.loginfo("Modem {} tock: {}".format(
                self.node_name, self.tock_topic_name))

        self.queue_tx_packet = rospy.Service(
            'queue_tx_packet', QueueTxPacket, self.handle_queue_tx_packet)

        # Wait for services to start before trying to process packets
        rospy.loginfo("Modem sim node waiting for simulation services")
        rospy.wait_for_service('/sim_transmission_loss')
        rospy.wait_for_service('/sim_packet_performance')
        rospy.wait_for_service('/sim_travel_time')
        self.call_sim_travel_time = rospy.ServiceProxy('/sim_travel_time', SimTravelTime)

        # Handling for sim to initiate and reply to Pings
        self.ping_reply_publisher = rospy.Publisher(
            'ping_reply', PingReply, queue_size=10)
        self.ping_modem_service = rospy.Service(
            'ping_modem', PingModem, self.handle_ping_modem)
        self.ping_transactions = {}
        self.ping_open_trans_cond_tbl = {}

        # AcousticRange publisher (on each rx publish range from src to rcv)
        self.acoustic_range_publisher = rospy.Publisher(
            '~acoustic_range', AcousticRange, queue_size=2, latch=True)

        # Set up a loop in a separate thread that handles received packets so that we can use the sim ticks
        self.rx_thread = Thread(target=self.process_rx_queue, daemon=True)
        self.tx_thread = Thread(target=self.process_outgoing_queue, daemon=True)
        self.sst_thread = Thread(target=self.sst_sim_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()
        self.sst_thread.start()

        # Create subscribers last so we don't have threads trying to use functionality that isn't initialized yet
        rospy.Subscriber('ambient_noise', Float32, self.on_ambient_noise)
        self.sim_subscriber = rospy.Subscriber(
            # '/acoustic_channel', SimPacket, self.on_acoustic_channel_sim_packet, tcp_nodelay=True)
            '/acoustic_channel', SimPacket, self.on_acoustic_channel_sim_packet)

        # Can be used to get the SRC address of the modem, and also serves as a mechanism to block until the acomms
        # simulator is up
        self.query_modem_src_service = rospy.Service(
            "query_modem_src", QueryModemSrc, self.handle_query_modem_src
        )

        # check if we should use the ping payload for message transport
        self.use_ping_payload = rospy.get_param("~use_ping_payload", False)
        # the default is the max bytes we can fit in a miniframe
        self.ping_maximum_miniframe_bytes = rospy.get_param("~ping_maximum_miniframe_bytes", 32)

        # if using the ping payload (or want to use later), configure the service proxy to message_queue
        rospy.loginfo("modem_sim_node creating get_next_packet_data service proxy")
        self.get_next_packet_data = rospy.ServiceProxy('get_next_packet_data', GetNextPacketData)

        rospy.loginfo("Modem sim node running{}. SRC={}, FC={} Hz, BW={} Hz".format(
                      " with sim ticks active" if self.use_sim_tick else "", self.src, self.fc, self.bw))

        rospy.spin()

    def handle_query_modem_src(self, query: QueryModemSrcRequest) -> QueryModemSrcResponse:
        response = QueryModemSrcResponse(src=self.src)
        return response

    def on_ambient_noise(self, msg: Float32):
        self.ambient_noise = msg.data

    def sst_sim_loop(self):
        noise_buffer = []
        while not rospy.is_shutdown():
            # We collect the ambient noise data and generate stats to match the SST message.
            # This isn't a perfect imitation, since it doesn't respect the state machine and doesn't account for interference
            rospy.sleep(0.5)
            noise_buffer.append(self.ambient_noise)
            if len(noise_buffer) > 20:
                noise_array = np.asarray(noise_buffer)
                sst_msg = SST(time=rospy.Time.now(),
                              sst_version=0,
                              in_water_spl_dB=self.ambient_noise,
                              detector=1,
                              num_samples=400,
                              summary_min=np.min(noise_array),
                              summary_lower_quartile=np.percentile(noise_array, 25),
                              summary_median=np.median(noise_array),
                              summary_upper_quartile=np.percentile(noise_array, 75),
                              summary_max=np.max(noise_array),
                              summary_len=250)
                noise_buffer.clear()
                self.sst_publisher.publish(sst_msg)

    def on_modem_location(self, msg: AnyMsg):
        msg = deserialze_anymsg(msg)
        rospy.logdebug(f"New modem location: latitude={msg.latitude}, longitude={msg.longitude}, depth={msg.depth}")
        self.modem_location = Position(msg.latitude, msg.longitude, msg.depth)

    def on_sim_tick(self, msg):
        try:
            self.sim_tick_queue.put_nowait(msg)
            if self.last_step_id > 0:
                if (msg.step_id != (self.last_step_id + 1)) and (msg.step_id != 0):
                    # TODO: check that step_id is sequential here (or reset to 0)
                    # if not squential, publish TOCK with 0x02
                    rospy.logerr_throttle(
                        1, "Sim tick step id jump ({} -> {})".format(self.last_step_id, msg.step_id))
            self.last_step_id = msg.step_id
        except Full:
            rospy.logerr_throttle(1, "Sim Tick overflow (last step id {}, incoming id {})".format(self.last_step_id,
                                                                                                  msg.step_id))
            # TODO: publish a TOCK with status=0x4
            # TODO: other errors, respond with generic reset command (0x01)

    def format_ping_for_tx_queue(self, dest, queue=3, packet_type=3):
        tx_packet = QueueTxPacket()
        tx_packet.queue = queue
        tx_packet.insert_at_head = False
        tx_packet.packet = Packet()
        tx_packet.packet.src = int(self.src)
        tx_packet.packet.dest = int(dest)
        tx_packet.packet.packet_type = int(packet_type)
        tx_packet.packet.miniframe_rate = 1
        tx_packet.packet.dataframe_rate = 1
        tx_packet.packet.miniframe_bytes = bytes()
        tx_packet.packet.dataframe_bytes = bytes()
        tx_packet.requested_src_level_db = 0

        return tx_packet

    def send_fdp_ping_sim(self, dest, rate=1, cdr=0, ping_payload=bytes()):
        queue_tx_packet_resp, sim_packet = None, None
        start_t = rospy.Time.now()

        queue_tx_packet_req = self.format_ping_for_tx_queue(dest=dest)
        queue_tx_packet_resp, sim_packet = self.handle_queue_tx_packet(
            queue_tx_packet_req=queue_tx_packet_req,
            ping={
                'ping_reply': self.create_ping_reply_msg(dest=dest),
                'ping_type': PingReplyTransaction.PING_TYPE_REQUEST,
                'transaction_start_t': start_t,
                'ping_payload': ping_payload,
            }
        )
        return queue_tx_packet_resp, sim_packet

    def create_ping_transaction(self, sim_packet):
        start_t = sim_packet.ping_transaction_start
        if start_t in self.ping_transactions:
            raise RuntimeError('''Error: In-complete ping transaction collision!
                Aborting this transaction... {}'''.format(self.ping_transactions[start_t]))

        self.ping_transactions[start_t] = PingReplyTransaction(
            transaction_start_t=start_t, event=Event())
        return self.ping_transactions[start_t]

    def handle_ping_modem(self, request):
        '''
        Send PingReply REQUEST to outgoing queue (handle_queue_tx_packet)
        Equivalent with a real modem would be sending $CCCMD,PNG,...
        '''
        rospy.logdebug("MODEM {}: Requesting modem send ping".format(self.src))
        ping_payload = request.hexdata

        # handle case where user wants to fill the ping payload with ros msgs
        if self.use_ping_payload and self.get_next_packet_data:
            rospy.loginfo(f"MODEM {self.src}: Attempting to fill ping payload for ROS msg transport. payload max size: {self.ping_maximum_miniframe_bytes} bytes")
            try:
                packet_data_response = self.get_next_packet_data(num_miniframe_bytes=self.ping_maximum_miniframe_bytes, num_dataframe_bytes=0)
            except:
                rospy.logwarn(f"ERROR: problem with get_next_packet_data() service. Not sending a payload with this ping...")
            else:
                if packet_data_response.num_messages > 0:
                    rospy.logdebug(f"Received data to send with ping request length: {len(packet_data_response.miniframe_bytes)}")
                    ping_payload = packet_data_response.miniframe_bytes
                else:
                    rospy.logdebug(f"Received NO data to send with ping request \npacket_data_response: {packet_data_response}\nlength: {len(packet_data_response.miniframe_bytes)}")

        try:
            # Equivalent with a real modem would be sending $CACMD,PNG,...
            queue_tx_packet_resp, sim_packet = self.send_fdp_ping_sim(request.dest,
                                                                      request.rate,
                                                                      request.cdr,
                                                                      ping_payload)
        except TypeError:
            import traceback
            rospy.logwarn("MODEM {}: Threw exception\r\n{}".format(
                self.src, traceback.format_exc()))
            rospy.logwarn("MODEM {}: dest: {}, {}".format(
                self.src, request.dest, repr(request.dest)))
            return PingModemResponse(timed_out=True)

        # if we failed to add to outgoing queue
        if not queue_tx_packet_resp:
            rospy.logwarn("Error! Request not sent:\n{}\n".format(request))
            # TODO should handle this error
            return PingModemResponse(timed_out=True)

        if request.timeout_sec < 1:
            request.timeout_sec = 1

        this_transaction, response, sim_packet_reply = None, None, None
        try:
            rospy.loginfo("MODEM {}: Starting transaction...".format(self.src))
            this_transaction = self.create_ping_transaction(
                sim_packet=sim_packet)
            # Equivalent with a real modem would be receiving $CACMR,PNR,...
            sim_packet_reply = this_transaction.wait(
                timeout=request.timeout_sec)

        except (TimeoutError, RuntimeError) as e:
            rospy.logdebug("Timeout in PingModem ROS Service call: {}".format(e))
        except ConnectionAbortedError as e:
            rospy.loginfo("PingModem ROS Service call cancelled: {}".format(e))
        except KeyError as e:
            rospy.logwarn("Unknown: {}".format(e))
        else:
            rospy.logwarn("MODEM {}: Received ping response!".format(self.src))

            ping_reply = sim_packet_reply.ping_reply
            transaction_duration = (this_transaction.transaction_end_t - this_transaction.transaction_start_t).secs
            ping_reply.tat = abs(transaction_duration - ping_reply.owtt)
            response = PingModemResponse(
                timed_out=False,
                one_way_travel_time=ping_reply.owtt,
                tat=ping_reply.tat,
                txlevel=ping_reply.tx_level,
                timestamp_resolution=ping_reply.timestamp_res,
                toa_mode=ping_reply.toa_mode,
                snv_on=ping_reply.snv_on,
                timestamp=ping_reply.timestamp,
                cst=ping_reply.cst,
            )
            self.ping_reply_publisher.publish(ping_reply)

        finally:
            if this_transaction:
                rospy.loginfo(
                    "MODEM {}: Closing transaction...".format(self.src))
                if this_transaction.transaction_end_t:
                    rospy.loginfo("MODEM {}: Transaction took {} seconds, requested timeout: {}".format(self.src,
                                                                                                        (this_transaction.transaction_end_t - this_transaction.transaction_start_t).secs,
                                                                                                        this_transaction.timeout))
                # this transaction either failed/succeeded, either way clean up
                del self.ping_transactions[this_transaction.transaction_start_t]
                if sim_packet_reply:
                    rospy.logdebug("MODEM {}: sim_packet_reply:\r\n{}".format(
                        self.src, sim_packet_reply))
            else:
                rospy.logwarn(
                    "MODEM {}: Transaction never created....".format(self.src))

            # if we got no response
            if not response:
                return PingModemResponse(timed_out=True)
            return response

    def create_ping_reply_msg(self, dest, sim_packet=None):
        ping_reply = PingReply(header=Header(stamp=rospy.Time.now()),
                               src=self.src,
                               dest=dest,
                               cst=CST(src=self.src, dest=dest))

        if sim_packet is not None:
            ping_reply.owtt = ping_reply.header.stamp.secs - sim_packet.ping_transaction_start.secs
        return ping_reply

    def handle_ping_rx(self, sim_packet):
        success = False
        if sim_packet.packet.dest != self.src:
            return sim_packet, success

        # needs to match senders transaction start time if we originated
        start_t = sim_packet.ping_transaction_start

        # this is a request from another modem not from the controlling computer
        # Equivalent with a real modem would be receiving a $CACMA,PNG,...
        if sim_packet.ping_type == PingReplyTransaction.PING_TYPE_REQUEST:
            rospy.logdebug("MODEM {}: Received request for a PingReply, sending...\r\n{}".format(
                self.src, sim_packet))
            dest = sim_packet.ping_reply.src
            queue_tx_packet_req = self.format_ping_for_tx_queue(dest=dest)

            rospy.logdebug('Created queue_tx_packet_req. About to send: \n{}\n{}\r\n{}'.format(
                start_t, dir(start_t), str(queue_tx_packet_req)))
            queue_tx_packet_resp, _ = self.handle_queue_tx_packet(
                queue_tx_packet_req=queue_tx_packet_req,
                ping={
                    'ping_reply': self.create_ping_reply_msg(dest=dest, sim_packet=sim_packet),
                    'ping_type': PingReplyTransaction.PING_TYPE_REPLY,
                    'transaction_start_t': start_t,
                })
            success = True

        # reply to request we made previously (or not?). Find transaction and close it out
        # Equivalent with a real modem would be receiving $CACMR,PNR,...
        elif sim_packet.ping_type == PingReplyTransaction.PING_TYPE_REPLY:
            rospy.logdebug("MODEM {}: Received reply! check if it's for us...\r\n{}".format(
                self.src, sim_packet))
            if start_t in self.ping_transactions.keys():
                # this ping is a match/response for an open transaction that we can now complete
                rospy.logdebug("MODEM {}: Completed Ping transaction! Closing...\r\n{}".format(
                    self.src, sim_packet))
                this_transaction = self.ping_transactions[start_t]
                success = this_transaction.complete_transaction(
                    sim_packet_reply=sim_packet)
            else:
                rospy.logdebug("MODEM {}: Received ping reply with no match. Publishing to ping_reply topic.\r\n{}".format(
                    self.src, sim_packet))
                success = False
                self.ping_reply_publisher.publish(sim_packet.ping_reply)
        else:
            success = False

        return sim_packet, success

    def process_rx_queue(self):
        try:
            while not rospy.is_shutdown():
                # If we are using sim ticks, block here until we get a tick
                if self.use_sim_tick:
                    # TODO: Clear everything on status = 0x0001
                    tick = self.sim_tick_queue.get()
                    if self.use_tick_time:
                        current_time = tick.header.stamp + tick.duration
                    else:
                        current_time = rospy.Time.now()
                else:
                    current_time = rospy.Time.now()

                rospy.logdebug("Processing timestep.")

                # Now, check to see if we have items in the queue.  We want to consume everything that is ready, but
                # not block.
                while True:
                    try:
                        rospy.logdebug("Trying to add to inwater queue")
                        new_packet = self.incoming_packets_queue.get_nowait()
                        self.packets_in_the_water.append(new_packet)
                        if self.verbose: rospy.loginfo("MODEM %d: Added new packet to inwater queue", self.src)
                    except Empty:
                        break

                rospy.logdebug("Processing inwater packets.")
                # Now, run the receive loop for this time tick
                self.process_inwater_packets(current_time=current_time)

                # loop through open ping transactions and see if any timed out!
                # important to do this here so sim time works correctly
                for start_t, transaction in self.ping_transactions.items():
                    if transaction.timeout and ((current_time - start_t).secs >= transaction.timeout):
                        # we have timed out. Close transaction
                        rospy.logdebug("MODEM {}: Stopping timed out transaction: {}".format(
                            self.src, transaction))
                        transaction.timed_out()

                if self.use_sim_tick:
                    # done, publish Tock message and wait for the next tick
                    if self.use_tick_time:
                        stamp = tick.header.stamp + tick.duration
                    else:
                        stamp = rospy.Time.now()
                    header = Header(stamp=stamp)
                    tock = Tock(header=header, node_name=self.node_name,
                                status=0, step_id=tick.step_id)
                    self.sim_tock_publisher.publish(tock)
                    rospy.logdebug("Published Tock {} for {}.".format(
                        tick.step_id, self.node_name))
                else:
                    # If we aren't using sim ticks, just sleep this thread for a bit
                    rospy.sleep(0.01)
        except rospy.ROSInterruptException:
            rospy.loginfo("MODEM %d: Shutdown process_rx_queue", self.src)

    def get_transmission_loss_sync(self, sim_packet: SimPacket, rcv_platform_location: Position):
        # fill SimTransmissionLossRequest
        transmission_loss_req = SimTransmissionLossRequest()
        transmission_loss_req.src_latitude = sim_packet.src_latitude
        transmission_loss_req.src_longitude = sim_packet.src_longitude
        transmission_loss_req.src_depth = sim_packet.src_depth
        transmission_loss_req.src_tx_level_db = sim_packet.src_tx_level_db
        transmission_loss_req.center_frequency_hz = sim_packet.center_frequency_hz
        transmission_loss_req.bandwidth_hz = sim_packet.bandwidth_hz
        transmission_loss_req.rcv_latitude = rcv_platform_location.latitude
        transmission_loss_req.rcv_longitude = rcv_platform_location.longitude
        transmission_loss_req.rcv_depth = rcv_platform_location.depth

        # call SimTransmissionLoss service to determine rcv_rx_level_dB & transmission_delay
        transmission_loss = self.call_sim_transmission_loss(
            transmission_loss_req)
        if not transmission_loss:
            rospy.logwarn("Error getting transmission loss using parameters:\n{}\n".format(
                transmission_loss_req))
            return

        recv_level = transmission_loss.rcv_rx_level_db  # receive source level
        arrival_time = transmission_loss.transmission_delay + sim_packet.transmit_time
        finish_time = arrival_time + sim_packet.transmit_duration
        arrival_time_str = str(datetime.datetime.utcfromtimestamp(arrival_time.to_sec()))
        src = (sim_packet.src_latitude, sim_packet.src_longitude)
        rcv = (rcv_platform_location.latitude, rcv_platform_location.longitude)
        rospy.loginfo(
                # "MODEM %d: T2: Receive level: %s Arrival time: %s SRC: (%s) RCV: (%s)",
                "MODEM %d: T2: Receive level: %s Arrival time: %s",
                self.src, f'{recv_level:.2f}', arrival_time_str, 
                # f'{src[0]:.2f}, {src[1]:.2f}, {sim_packet.src_depth:.2f}', 
                # f'{rcv[0]:.2f}, {rcv[1]:.2f}, {rcv_platform_location.depth:.2f}'
            )
        try:
            owtt = (arrival_time - sim_packet.transmit_time).to_sec()
            if owtt <= 0:
                rospy.logwarn(f'WARNING: reception with OWTT <= 0! owtt: {owtt}. Setting to 0.00066 for range calc.. (1m @ 1500m/s)')
                owtt = 0.00066
            distance_m = geopy_distance(src, rcv, ellipsoid='WGS-84').meters
            sound_speed = distance_m / owtt

            rospy.loginfo(
                "MODEM %d: T2: SRC: (%s) RCV: (%s), Distance (km): %s, OWTT: %s Sound Speed (m/s): %s", 
                self.src,
                f'{src[0]:.2f}, {src[1]:.2f}, {sim_packet.src_depth:.2f}', 
                f'{rcv[0]:.2f}, {rcv[1]:.2f}, {rcv_platform_location.depth:.2f}',
                f'{distance_m / 1000:.2f}', f'{owtt:.2f}', f'{sound_speed:.2f}',
            )
        except:
            # TODO: we don't want to spam the user with warnings but, we need to handle the case where location params are,
            # .. out of range/invalid. we handle owtt <= 0 but, other cases should potentially be handled here.
            pass
        else:
            # create acoustic range for this sim_packet
            measurement_type = AcousticRange.MEASUREMENT_TYPE_OWTT
            # check if this is a ping packet
            if sim_packet.ping_type > PingReplyTransaction.PING_TYPE_NONE:
                # measurement_type = AcousticRange.MEASUREMENT_TYPE_OWTT if sim_packet.is_transponder else AcousticRange.MEASUREMENT_TYPE_TWTT
                measurement_type = AcousticRange.MEASUREMENT_TYPE_TWTT # pretty sure this is the only type we have for sim

            acoustic_range_msg = AcousticRange(header=Header(stamp=rospy.Time.now()),
                                               measurement_type=measurement_type,
                                               speed_of_sound_source=AcousticRange.SPEED_OF_SOUND_SOURCE_INVERTED_FROM_DISTANCE,
                                               src=sim_packet.packet.src,
                                               dest=sim_packet.packet.dest,
                                               owtt=owtt,
                                               range_m=distance_m,
                                               speed_of_sound_ms=sound_speed)
            self.acoustic_range_publisher.publish(acoustic_range_msg)

        return recv_level, arrival_time, finish_time

    def check_if_packet_is_active(self, current_time: rospy.Time,
                                  sim_packet: SimPacket,
                                  rcv_platform_location: Position) -> Optional[ActivePacket]:

        # We first check the arrival time, which doesn't require calculating receive level (and therefore doesn't
        # necessarily require running bellhop
        response = self.call_sim_travel_time(SimTravelTimeRequest(src_latitude=sim_packet.src_latitude,
                                                                  src_longitude=sim_packet.src_longitude,
                                                                  src_depth=sim_packet.src_depth,
                                                                  rcv_latitude=rcv_platform_location.latitude,
                                                                  rcv_longitude=rcv_platform_location.longitude,
                                                                  rcv_depth=rcv_platform_location.depth,
                                                                  center_frequency_hz=sim_packet.center_frequency_hz))
        arrival_time = sim_packet.transmit_time + response.travel_time

        # If the packet hasn't arrived yet, we're done with this packet for now.
        if arrival_time > current_time:
            return None

        # Once the packet arrives, we need receive level.
        recv_level, arrival_time, finish_time = self.get_transmission_loss_sync(
            sim_packet, rcv_platform_location)

        # If this packet is in progress, we need to add it to the list of active packets.
        # This list short be sorted, by virtue of the order in which things are added.
        active_packet = ActivePacket(
            sim_packet, arrival_time, finish_time, recv_level)
        return active_packet

    def create_rx_packet(self, finished_packet: ActivePacket, total_noise: float) -> ReceivedPacket:
        # if packet succeeds, publish received packet
        received_packet = ReceivedPacket()
        received_packet.packet = finished_packet.sim_packet.packet
        # Not sure if I should do arrival time or just Time.now()
        received_packet.header.stamp = finished_packet.arrival_time
        # Need to actually fill in CST field
        received_packet.cst.src = received_packet.packet.src
        received_packet.cst.dest = received_packet.packet.dest
        received_packet.cst.packet_type = 0 if finished_packet.sim_packet.ping_type > PingReplyTransaction.PING_TYPE_NONE else 5
        received_packet.cst.agn = 250
        received_packet.cst.bandwidth = self.bw
        received_packet.cst.carrier = self.fc
        received_packet.cst.mode = 0
        received_packet.cst.data_rate = received_packet.packet.dataframe_rate
        received_packet.cst.rate_num = received_packet.packet.miniframe_rate
        received_packet.cst.mfd_spl = int(finished_packet.receive_level_db)
        received_packet.cst.snr_in = finished_packet.receive_level_db - total_noise
        return received_packet

    def create_tx_packet(self, transmitted_packet: TransmittedPacket) -> TransmittedPacket:
        # Need to actually fill in XST field
        transmitted_packet.xst.src = transmitted_packet.packet.src
        transmitted_packet.xst.dest = transmitted_packet.packet.dest
        transmitted_packet.xst.packet_type = 5
        transmitted_packet.xst.bandwidth = self.bw
        transmitted_packet.xst.carrier = self.fc
        transmitted_packet.xst.mode = 0
        transmitted_packet.xst.rate_num = transmitted_packet.packet.miniframe_rate
        return transmitted_packet

    def get_truncated_frames(self, frame_success_mask: list, frame_bytes, hit_first_bad_frame=False):
        truncated_frame = []
        num_bad_frames = 0

        tmp_frame_success_mask = []
        mask_multiplier = len(frame_bytes) // len(frame_success_mask)

        for frame_good in frame_success_mask:
            tmp_frame_success_mask.extend([frame_good] * mask_multiplier)
        frame_success_mask = tmp_frame_success_mask

        for frame_good, frame_data in zip(frame_success_mask, frame_bytes):
            if frame_good and not hit_first_bad_frame:
                truncated_frame.append(frame_data)
            else:
                hit_first_bad_frame = True
                num_bad_frames += 1

        return bytes(truncated_frame), num_bad_frames, frame_success_mask

    def handle_partial_packet(self, packet_performance: SimPacketPerformanceResponse, sim_packet: SimPacket):
        # if the packet_performance says packet failed (some frames were bad), find the first bad frame and truncate the packet at that bad frame
        if not packet_performance.packet_success:
            rospy.logdebug(f'packet_performance: {packet_performance}')
            rospy.logdebug(f'Before truncating partial packet: \nmini: {list(sim_packet.packet.miniframe_bytes)}\ndata: {list(sim_packet.packet.dataframe_bytes)}')
            rospy.logdebug(f'frame success mask: \nmini: {list(packet_performance.miniframe_success)}\ndata: {list(packet_performance.frame_success)}')
            # miniframe
            sim_packet.packet.miniframe_bytes, num_bad_mini_frames, miniframe_success_mask = self.get_truncated_frames(
                    frame_success_mask=packet_performance.miniframe_success,
                    frame_bytes=sim_packet.packet.miniframe_bytes,
                )
            # dataframe
            # For now, don't return any dataframe bytes if we don't have all good miniframes
            if num_bad_mini_frames == 0:
                sim_packet.packet.dataframe_bytes, num_bad_data_frames, dataframe_success_mask = self.get_truncated_frames(
                        frame_success_mask=packet_performance.frame_success,
                        frame_bytes=sim_packet.packet.dataframe_bytes,
                        hit_first_bad_frame=bool(num_bad_mini_frames),
                    )
            else:
                # return an empty set of dataframes
                sim_packet.packet.dataframe_bytes = bytes()
            # rospy.loginfo(f'Length of miniframe and miniframe mask: {len(sim_packet.packet.miniframe_bytes)}, {len(miniframe_success_mask)}')
            # rospy.loginfo(f'Length of dataframe and dataframe mask: {len(sim_packet.packet.dataframe_bytes)}, {len(dataframe_success_mask)}')
            rospy.logdebug(f'After truncating partial packet: \nmini (bad frames: {num_bad_mini_frames}): {list(sim_packet.packet.miniframe_bytes)}\ndata (bad frames: {num_bad_data_frames}): {list(sim_packet.packet.dataframe_bytes)}')

        success = bool(len(sim_packet.packet.dataframe_bytes) or len(sim_packet.packet.miniframe_bytes))
        return sim_packet, success

    def process_inwater_packets(self, current_time) -> None:
        rcv_platform_location = self.modem_location

        # If we are using messages to get location, we may not have received a message yet.
        if rcv_platform_location.latitude is None:
            return

        rospy.logdebug("MODEM {}: location is {}".format(
            self.src, rcv_platform_location))

        # # Look at all the packets in the water and see if any of them are being received right now.
        for packet_in_the_water in self.packets_in_the_water:
            try:
                active_packet = self.check_if_packet_is_active(current_time, packet_in_the_water, rcv_platform_location)
            except TypeError:
                rospy.logerr(
                    "Error checking if packet is active (Bellhop failure?), may have missed a packet")
                active_packet = None

            if active_packet:
                self.packets_in_the_water.remove(packet_in_the_water)
                self.active_rx_packets.append(active_packet)
                if self.verbose: rospy.loginfo("MODEM {}: Incoming packet receive start".format(self.src))

        # If there are no active packets, we are done with this cycle.
        if not self.active_rx_packets:
            return

        # Clean up the list of leftover packets.  Remove any packets that finished before any of our currently active
        # packets started.
        latest_start_time = max([packet.arrival_time for packet in self.active_rx_packets])
        for leftover_packet in list(self.leftover_rx_packets):
            if leftover_packet.finish_time < latest_start_time:
                self.leftover_rx_packets.remove(leftover_packet)

        # Now, look through the active packets and see how we're doing.
        # If it's in the active list, then we have started receiving, but haven't finished yet.
        # See if any of them finished in this cycle
        finished_packets = [
            packet for packet in self.active_rx_packets if packet.finish_time <= current_time]
        if not finished_packets:
            return

        if self.verbose: rospy.loginfo("MODEM %d: Processing finished packets", self.src)

        for finished_packet in finished_packets:
            # Our noise is an incoherent sum of the receive level of all of the other packets, plus the user-specified
            # ambient noise.
            other_packets = list(self.active_rx_packets)
            other_packets.extend(
                [packet for packet in self.leftover_rx_packets if packet.finish_time > finished_packet.arrival_time])
            other_packets.remove(finished_packet)
            noise_levels = [
                packet.receive_level_db for packet in other_packets]
            noise_levels.append(self.ambient_noise)
            total_noise = sum_incoherent_db(noise_levels)

            # call packet performance on this packet
            packet_performance_req = SimPacketPerformanceRequest()
            packet_performance_req.rx_level_db = finished_packet.receive_level_db
            packet_performance_req.noise_level_db = total_noise
            packet_performance_req.packet_rate = finished_packet.sim_packet.packet.dataframe_rate
            packet_performance = self.call_sim_packet_performance(packet_performance_req)

            dest = finished_packet.sim_packet.packet.dest
            if packet_performance:
                if packet_performance.packet_success:
                    rospy.loginfo(f"MODEM {self.src}: T3: Packet succeeded, preparing received_packet")
                    sim_packet, success = finished_packet.sim_packet, packet_performance.packet_success
                else:
                    rospy.loginfo(f"MODEM {self.src}: T3: Packet failed, recovering good frames then preparing received_packet")
                    # like the acomms driver node, this will truncate the packet at first bad frame
                    sim_packet, success = self.handle_partial_packet(packet_performance=packet_performance, sim_packet=finished_packet.sim_packet)

                # check if this is a ping packet so we can handle additional
                # requested actions (reply to a request, request a reply)
                if sim_packet.ping_type != PingReplyTransaction.PING_TYPE_NONE:
                    sim_packet, success = self.handle_ping_rx(sim_packet=sim_packet)

                finished_packet = ActivePacket(sim_packet=sim_packet,
                                               arrival_time=finished_packet.arrival_time,
                                               finish_time=finished_packet.finish_time,
                                               receive_level_db=finished_packet.receive_level_db)
                if success:
                    self.publish_received_packet(self.create_rx_packet(
                        finished_packet=finished_packet, total_noise=total_noise))
                else:
                    rospy.logdebug("MODEM {}: Packet: {}, Packet not for us".format(
                        self.src, dest))
                    rospy.logdebug("MODEM {}: Packet: {}, Packet not for us\r\n{}".format(
                        self.src, dest, finished_packet))

            else:
                rospy.loginfo(
                    "MODEM %d: Packet: %d T3: Packet not received", self.src, dest)

            # Now, move the finished packet out of the list of active packets, but keep it around in case it
            # is interfering with another packet that isn't done yet.
            # We clean up the list of leftover packets in the next cycle.
            self.active_rx_packets.remove(finished_packet)
            self.leftover_rx_packets.append(finished_packet)

    """ Subscribes to SIM Packets """

    def on_acoustic_channel_sim_packet(self, sim_packet):
        rospy.loginfo("MODEM {}: Heard new packet on acoustic channel. dest: {}".format(
            self.src, sim_packet.packet.dest))
        # check if packet is from self or another modem
        if self.src != sim_packet.packet.src:
            # check if in band - if not in band it is not added to queue
            if self.packet_in_band(sim_packet):
                if self.verbose: rospy.loginfo("MODEM %d: Adding packet to incoming_packets_queue", self.src)
                self.incoming_packets_queue.put_nowait(sim_packet)
            else:
                rospy.loginfo(
                    "MODEM %d: Stopped receive_sim_packet, out of Band", self.src)
        else:
            rospy.loginfo(
                "MODEM %d: Stopped receive_sim_packet, own packet", self.src)

    """Checks if the packet is in the modem band"""

    def packet_in_band(self, sim_packet):
        min_f_m = int(round(self.fc - self.bw / 2))
        max_f_m = int(round(self.fc + self.bw / 2))
        min_f_p = int(round(sim_packet.center_frequency_hz -
                      sim_packet.bandwidth_hz / 2))
        max_f_p = int(round(sim_packet.center_frequency_hz +
                      sim_packet.bandwidth_hz / 2))
        len_range = len(range(max(min_f_m, min_f_p), min(max_f_m, max_f_p)))
        if len_range:
            return True
        else:
            return False

    """ Provides QueueTxPacket Service """

    def handle_queue_tx_packet(self, queue_tx_packet_req, ping=None):
        if self.verbose: rospy.loginfo("MODEM %d: Starting handle_queue_tx_packet", self.src)
        # Input: queue, insert_at_head, packet, requested_src_level_dB
        sim_packet = SimPacket()
        queue_tx_packet_resp = QueueTxPacketResponse()

        '''
        This will add data needed for a sim_packet that includes a ping
        both PingReplyTransaction.PING_TYPE_REQUEST and PingReplyTransaction.PING_TYPE_REPLY
        work their way through this method
        '''
        if ping:
            sim_packet.ping_reply = ping['ping_reply']
            sim_packet.ping_type = ping['ping_type']
            sim_packet.ping_transaction_start = ping['transaction_start_t']
            # optionally user can include a payload the size of a miniframe
            miniframe_bytes = ping.get('ping_payload', None)
        else:
            sim_packet.ping_type = PingReplyTransaction.PING_TYPE_NONE
            miniframe_bytes = None # unset, use packet from req

        sim_packet.packet = queue_tx_packet_req.packet
        sim_packet.packet.src = self.src
        if miniframe_bytes is not None:
            # only set if explicit ping_payload key in req
            sim_packet.packet.miniframe_bytes = miniframe_bytes

        if queue_tx_packet_req.requested_src_level_db == 0:
            queue_tx_packet_req.requested_src_level_db = 180
        queue_tx_packet_resp.actual_src_level_db = self.get_actual_src_level_db(
            queue_tx_packet_req.requested_src_level_db)
        sim_packet.src_tx_level_db = queue_tx_packet_resp.actual_src_level_db

        # TODO: Do we need to handle insert at head here?
        self.packet_transmit_queue.put_nowait(sim_packet)
        queue_tx_packet_resp.position_in_queue = self.packet_transmit_queue.qsize() - 1
        queue_tx_packet_resp.success = True
        if self.verbose: rospy.loginfo("MODEM %d: Adding Sim Packet to Queue at tail", self.src)

        # Reply: success, position_in_queue, actual_src_level_db
        # if this is a sim_packet that includes a ping, we return the sim_packet we made
        # this sim_packet will share some of it's attrs when we create the PingReplyTransaction
        if ping:
            return queue_tx_packet_resp, sim_packet
        return queue_tx_packet_resp
    
    """ Calls SimPacketPerformance Service """

    def call_sim_packet_performance(self, sim_packet_performance_req):
        try:
            sim_packet_performance = rospy.ServiceProxy(
                '/sim_packet_performance', SimPacketPerformance)
            packet_performance = sim_packet_performance(
                sim_packet_performance_req)
            log_str = "MODEM %d: SimPacketPerformanceReply: Packet Succes %r" % (self.src, packet_performance.packet_success)
            if self.verbose: rospy.loginfo(log_str)
            return packet_performance
        except rospy.ServiceException:
            rospy.loginfo("MODEM %d: Sim Packet Performance Service call failed", self.src)

    """ Calls SimTransmissionLoss Service """

    def call_sim_transmission_loss(self, sim_transmission_loss_req):
        try:
            sim_transmission_loss = rospy.ServiceProxy(
                '/sim_transmission_loss', SimTransmissionLoss)
            transmission_loss = sim_transmission_loss(
                sim_transmission_loss_req)
            log_str = "MODEM %d: SimTransmissionLossReply: Recv Level [dB]: %f, Transmission delay: %f" % (
                self.src,
                transmission_loss.rcv_rx_level_db,
                transmission_loss.transmission_delay.to_sec())
            rospy.logdebug(log_str)
            return transmission_loss
        except rospy.ServiceException:
            rospy.loginfo(
                "MODEM %d: Sim Transmission Loss Service call failed", self.src)

    """ Take in requested src_level_db and return actual """

    def get_actual_src_level_db(self, src_level_requested):
        # Actual SRC levels provided by Keenan
        # levels_25kHz = [185,181,179,167]
        # levels_10kHz = [183,179,177,165]

        return src_level_requested

    """ Checks for SimPackets to transmit, transmits  """

    def process_outgoing_queue(self):
        try:
            while not rospy.is_shutdown():
                # get platform location
                platform_location = self.modem_location
                if platform_location.latitude is None:
                    # This can happen if the source is 'message' and no message has been published yet
                    # Wait for actual wall-clock time, since this isn't really part of the simulation, and we can end
                    # up with an undesired lock here if the time source is waiting on us for some reason.
                    rospy.sleep(0.05)
                    continue

                # if there is a sim packet in the queue
                sim_packet = self.packet_transmit_queue.get()
                if self.verbose: rospy.loginfo("MODEM %d: T1: Popping Queued SimPacket for transmission", self.src)

                # fill in remaining sim_packet parameters
                sim_packet.src_latitude = platform_location.latitude
                sim_packet.src_longitude = platform_location.longitude
                sim_packet.src_depth = platform_location.depth
                sim_packet.center_frequency_hz = self.fc
                sim_packet.bandwidth_hz = self.bw
                packet_time = self.get_packet_time(sim_packet)
                sim_packet.transmit_duration = rospy.Duration(packet_time)
                sim_packet.transmit_time = rospy.Time.now()

                # call publish_sim
                self.publish_sim(sim_packet)

                # call transmitted_packet
                transmitted_packet = TransmittedPacket()
                # Not sure if I should do transmit time or just Time.now()
                transmitted_packet.header.stamp = sim_packet.transmit_time
                transmitted_packet.packet = sim_packet.packet
                # fill out the XST part
                transmitted_packet = self.create_tx_packet(transmitted_packet)
                # correct packet type for case of ping
                if sim_packet.ping_type > PingReplyTransaction.PING_TYPE_NONE:
                    # this XST is a ping
                    transmitted_packet.xst.packet_type = 0

                # publish_transmitted_packet
                self.publish_transmitted_packet(transmitted_packet)
        except rospy.ROSInterruptException:
            rospy.loginfo(
                "MODEM %d: Shutdown process_outgoing_queue", self.src)

    """Get the time the packet takes to transmit"""

    def get_packet_time(self, sim_packet):
        # use the fc, bw, minipacket rate, packet rate, data to calc transmit time
        if sim_packet.ping_type > PingReplyTransaction.PING_TYPE_NONE:
            return 0.5
        return 5  # just a placeholder

    """ Publishes SimPacket """

    def publish_sim(self, sim_packet):
        # Publish a SimPacket message to the sim topic
        if self.verbose: rospy.loginfo("MODEM {}: Packet {}: Publishing SimPacket msg to acoustic_channel topic".format(self.src, sim_packet.packet.dest))
        self.sim_publisher.publish(sim_packet)

    """ Publishes ReceivedPacket """

    def publish_received_packet(self, received_packet: ReceivedPacket):
        # Publish a ReceivedPacket message and the associated CST message

        # Send source ID

        rospy.loginfo("MODEM {}: Packet {}: Publishing ReceivedPacket msg to received topic".format(
            self.src, received_packet.packet.dest))
        self.rx_publisher.publish(received_packet)
        self.cst_publisher.publish(received_packet.cst)

    """ Publishes TransmittedPacket """

    def publish_transmitted_packet(self, transmitted_packet):
        # Publish and TransmittedPacket message to xst topic

        # Send source ID

        rospy.loginfo("MODEM {}: Packet {}: Publishing TransmittedPacket msg to transmitted topic".format(
            self.src, transmitted_packet.packet.dest))
        self.tx_publisher.publish(transmitted_packet)
        self.xst_publisher.publish(transmitted_packet.xst)

if __name__ == '__main__':
    try:
        node = ModemSimNode()
        rospy.loginfo("Modem Sim Node shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("Modem Sim Node shutdown (interrupt)")
