#!/usr/bin/env python3
from __future__ import unicode_literals, annotations

import traceback
from itertools import groupby
from typing import Any, Deque, Dict, List, Optional, Tuple, Union

import rospy
from rospy.msg import AnyMsg
from std_msgs.msg import Header
from functools import partial
from importlib import import_module
from collections import namedtuple, deque, defaultdict
from dataclasses import dataclass
from datetime import datetime, timedelta
import time
from ros_acomms_msgs.srv import GetNextPacketData, GetNextPacketDataRequest, GetNextPacketDataResponse
from ros_acomms_msgs.srv import GetNextQueuedMessage, GetNextQueuedMessageRequest, GetNextQueuedMessageResponse
from ros_acomms_msgs.srv import QueryModemSrc, QueryModemSrcRequest, QueryModemSrcResponse
from ros_acomms_msgs.msg import UpdateQueuedMessage
from ros_acomms_msgs.msg import (NeighborReachability, QueueEnable,
                            QueuePriority, QueueStatus, QueueSummary, SummaryMessageCount)
from bitstring import Bits, BitStream, BitArray
from threading import Lock
import sys
import os
import signal

from ltcodecs import RosMessageCodec, EncodingFailed
from acomms_codecs import packet_codecs

from version_node import version_node

from codec_config_parser import (QueueParams, get_queue_params, get_message_codecs, read_packet_codec_yaml,
                                 load_custom_codecs)

from mac_utils import TagConfig

@dataclass
class DynamicQueue:
    dest_address: int
    message_codec_id: int  # the ID from the message codec params
    message_codec: RosMessageCodec
    dynamic_message_service: str
    dynamic_update_topic: Optional[str] = None


@dataclass
class QueuedMessage:
    queue: Any  # Should be MessageQueue, but that's not defined yet.
    message_codec: RosMessageCodec
    message: AnyMsg
    dest: int
    payload_bits: Bits
    length_bits: int
    time_added: datetime
    message_id_in_queue: Optional[int] = None
    transmitted: bool = False
    sequence_number: Optional[int] = None
    dynamic_queue_name: Optional[str] = None

    @property
    def priority(self) -> int:
        return self.queue.priority

    def get_next_payload_smaller_than_bits(self, size_in_bits: int,
                                           dest_sequence_nums,
                                           remove_from_queue: bool = False
                                           ) -> Tuple[Optional[Bits], Optional[Bits]]:

        rospy.logdebug("Getting next bits up to: {0}".format(size_in_bits))
        rospy.logdebug("Self lengthbits: {0}".format(self.length_bits))

        if self.length_bits > size_in_bits:
            rospy.logwarn(f"Got {self.length_bits} bits, but we can only fit {size_in_bits}")
            return None, None

        if self.length_bits == 0:
            rospy.logerr("Message payload length is 0, which shouldn't happen")
            return None, None

        message_bits = self.payload_bits
        message_header = self.queue.message_codec_id

        rospy.logdebug("Returning {0} of {1} bits".format(message_bits.length, self.payload_bits.length))
        if remove_from_queue and isinstance(self.queue, MessageQueue):
            self.queue.mark_transmitted(self)

        return message_header, message_bits


class MessageQueue(object):
    def __init__(self,
                 master_message_queue: List[QueuedMessage],
                 dest_address: int,
                 message_codec_id: int,  # the ID from the message codec params
                 topic: str,
                 message_codec: RosMessageCodec,
                 priority: int = 1,
                 order: str = 'lifo',
                 maxsize: int = 100,
                 dynamic_message_service: Optional[str] = None,
                 ) -> None:
        self.master_message_queue = master_message_queue
        self.dynamic_message_service = dynamic_message_service
        self.order = order
        self.dest_address: int = dest_address
        self.priority: int = priority
        self.message_codec_id = message_codec_id
        self.subscribe_topic: str = topic
        self.message_codec = message_codec

        self._last_tx_time = datetime.utcnow()
        self._queue: Deque[QueuedMessage] = deque(maxlen=maxsize)

    def priority_update(self, priority_requested: int) -> int:
        rospy.logwarn("Priority request for msg queue: {}".format(priority_requested))
        self.priority = priority_requested
        return self.priority

    def append(self, message: AnyMsg) -> None:
        try:
            encoded_bits, metadata = self.message_codec.encode(message)
        except EncodingFailed as e:
            rospy.logwarn(f"Failed to encode message: {e.message} (message is {message}, exception is {e})")
            return

        rospy.logdebug("Append: {}".format(encoded_bits))
        dest = self.dest_address
        if metadata:
            if 'dest' in metadata:
                dest = metadata['dest']

        queued_message = QueuedMessage(message=message,
                                       message_codec=self.message_codec,
                                       queue=self,
                                       dest=dest,
                                       payload_bits=encoded_bits,
                                       length_bits=encoded_bits.length,
                                       time_added=datetime.utcnow(),
                                       transmitted=False,
                                       )

        if self.order == 'fifo':
            if len(self._queue) < self._queue.maxlen:
                # drop this message if the queue is full.
                self._queue.appendleft(queued_message)
                # master message queue draws from the top
                self.master_message_queue.append(queued_message)
        else:
            # For LIFO, we threw away the old messages
            if len(self._queue) == self._queue.maxlen:
                # drop the oldest message
                message_to_remove = self._queue.popleft()
                self.master_message_queue.remove(message_to_remove)
            self._queue.append(queued_message)
            self.master_message_queue.insert(0, queued_message)

        rospy.logdebug_throttle(5, "Added message to queue, {} messages".format(len(self._queue)))

    def mark_transmitted(self, queued_message):
        # type: (QueuedMessage) -> None
        if queued_message in self._queue:
            queued_message.transmitted = True
            self._last_tx_time = datetime.utcnow()
            self._queue.remove(queued_message)
            self.master_message_queue.remove(queued_message)

    def get_next_message(self, remove_from_queue: bool = True) -> Optional[QueuedMessage]:
        if len(self._queue) == 0:
            return None
        queued_message = self._queue[-1]
        if remove_from_queue:
            self._queue.pop()
            self._last_tx_time = datetime.utcnow()
        return queued_message

    def get_next_message_smaller_than_bits(self, size_in_bits: int, remove_from_queue: bool = False) -> Optional[QueuedMessage]:
        if len(self._queue) == 0:
            return None
        for qm in reversed(self._queue):
            if qm.length_bits <= size_in_bits:
                if remove_from_queue:
                    self._queue.remove(qm)
                    self._last_tx_time = datetime.utcnow()
                return qm
        return None

    @property
    def bit_count(self) -> int:
        count = 0
        for msg in self._queue:
            count += msg.length_bits
        return count

    @property
    def message_count(self) -> int:
        return len(self._queue)

    @property
    def time_since_last_tx(self) -> timedelta:
        # TODO: make this work with sim time
        if self.message_count == 0 or not self._last_tx_time:
            return timedelta(seconds=0)
        else:
            return datetime.utcnow() - self._last_tx_time


class MessageQueueNode(object):
    """
    Message queue node.  Manages all static queues as well as connections to dynamic queues.

    All attributes are set with rosparams.

    Params:
        update_rate: Rate to publish status messages (in Hz)
        unknown_dests_are_reachable:
    """
    def __init__(self) -> None:
        rospy.init_node('message_queue')
        self.update_rate = rospy.get_param('~update_rate', 1)
        self.unknown_dests_are_reachable: bool = rospy.get_param('~unknown_dests_are_reachable', True)
        self.default_dest = rospy.get_param('~default_dest', 121)
        self.default_priority = rospy.get_param('~default_priority', 10)

        src_param_name = rospy.search_param('src')
        if src_param_name:
            self.src = rospy.get_param(src_param_name)
        else:
            rospy.loginfo("src parameter was not found.  Querying src ID from modem... (Requires that modem driver or sim node is up)")
            try:
                rospy.wait_for_service('query_modem_src')
                query_modem_src = rospy.ServiceProxy('query_modem_src', QueryModemSrc)
                src_response: QueryModemSrcResponse = query_modem_src()
                self.src = src_response.src
            except rospy.ServiceException as e:
                rospy.logerr("Error calling query_modem_src service.  Setting src to -1, which may break packet codecs.")
                self.src = -1

        try:
            version = version_node()
            version.getLtCodecs()
            version.getAcomms()
        except:
            rospy.logwarn("Unable to query version information")

        # Init dynamic queues
        self.dynamic_queues: dict[str, DynamicQueue] = {}
        self.dynamic_get_next_message_services = {}
        self.dynamic_queue_update_publishers = {}

        # This needs to be called before any subscribers are created
        self.setup_queues()

        # Status publishers
        self.queue_status_pub = rospy.Publisher('queue_status', QueueStatus, queue_size=10, latch=True)

        # Queue priority/enable update Subscriptions
        self.priority_sub = rospy.Subscriber("queue_priority", QueuePriority, self.handle_queue_priority)
        self.enable_sub = rospy.Subscriber("queue_enable", QueueEnable, self.handle_queue_enable)

        # Subscribe to the Neighbor Reachability updates (which may be published by more than one node
        rospy.Subscriber("neighbor_reachability", NeighborReachability, self.on_neighbor_reachability)

        # We use a lock to ensure that we handle only one get_next_packet_data call at a time
        self.get_next_packet_data_lock = Lock()
        self.acquire_lock_attempt_count = 0
        self.acquire_lock_max_attempts = rospy.get_param('~acquire_lock_max_attempts', 10)
        self.current_packet_dynamic_queue_cache: dict[str, GetNextQueuedMessageResponse] = {}
        self.current_packet_tag_config: TagConfig = TagConfig()
        self.get_packet_data_service = rospy.Service('get_next_packet_data', GetNextPacketData,
                                                     self.handle_get_next_packet)

        rospy.loginfo("Message Queue started")
        self.spin()

    def setup_queues(self) -> None:
        self.topic_queue = {}
        self.topic_headers = {}

        self.destination_reachable = {}
        self.dest_sequence_num = defaultdict(lambda: 1)

        self.queues_by_priority = defaultdict(list)
        self._queued_messages: List[QueuedMessage] = []

        self.active_queue_names = set()

        # Load custom codecs.  We do this here because we can't really get rosparams until we have a node.
        rospy.loginfo(f'Loading Custom Codecs...')
        try:
            load_custom_codecs()
        except Exception as e:
            rospy.logwarn(f'WARNING: Problem Loading Custom Codecs: {(e,)}', exc_info=True)

        rospy.loginfo(f'Reading Packet Codecs...')
        try:
            packet_codec_param = read_packet_codec_yaml()
        except Exception as e:
            rospy.logwarn(f'WARNING: Problem Reading Packet Codecs: {(e,)}', exc_info=True)
            packet_codec_param = []

        rospy.loginfo(f'Building Packet/Message Codecs...')
        # QUESTION(lindzey): Do we ever have more than one packet codec? I'm
        #   not convinced that the code in this block actually handles that case
        #   appropriately, unless different packet codecs are guaranteed to
        #   have non-overlapping destinations and subscribe topics.
        # WARNING: This dict will work so long as multiple packet codecs have
        #   non-overlapping lists of QueueIDs.
        self.queue_params: Dict[int, QueueParams] = {}
        for packet_codec_details in packet_codec_param:
            rospy.loginfo("Packet codec details: {}".format(packet_codec_details))
            packet_codec_class = packet_codecs[packet_codec_details['packet_codec']]  # type: PacketCodec

            self.queue_params.update(get_queue_params(packet_codec_details))

            try:
                message_codecs = get_message_codecs(packet_codec_details)
            except Exception as e:
                rospy.logerr(f'ERROR Getting Message Codecs: {(e,)}')
                raise e

            packet_codec_details['message_codecs'] = message_codecs

            # Convert SRC IDs to integers, since yaml dict keys have to be strings
            try:
                src_names: dict = {int(k):v for k,v in packet_codec_details.get('src_names', {}).items()}
            except Exception as e:
                rospy.logerr(f"Error parsing src_names parameter: {e}")

            for queue_id, params in self.queue_params.items():
                # We support three cases here: Publish only, subscribe to a topic, or call a dynamic queue service
                # If there is no subscribe_topic or dynamic queue, this is a publish-only entry, so we skip it here
                if params.subscribe_topic is None and params.dynamic_queue_service is None:
                    rospy.logdebug(f"ID {queue_id} has no subscribe topic or dynamic queue, skipping it.")
                    continue
                # If both the subscribe_topic and dynamic_queue are specified, that's bad.
                # Throw an error (but don't crash for now)
                if params.subscribe_topic and params.dynamic_queue_service:
                    rospy.logerr(f"ID {queue_id} has both a subscribe_topic and dynamic_queue specified.  You can't have both.")
                    continue

                # If no destination is specified on this queue, fill in our global default
                if params.destination is None:
                    params.destination = self.default_dest

                # If no priority is specified on this queue, fill in our global default
                if params.priority is None:
                    params.priority = self.default_priority

                self.destination_reachable[params.destination] = self.unknown_dests_are_reachable

                # Now, the "normal" case: the subscribe_topic is specified, so we want to set up a new queue for it.
                if params.subscribe_topic:
                    topic_name = params.subscribe_topic
                    if '{src}' in topic_name or '{name}' in topic_name:
                        # We are going to attach multiple subscribers to this queue, one for each entry in the
                        # src_names dict
                        for src, name in src_names.items():
                            complete_topic_name = topic_name.format(src=src, name=name)
                            self.init_static_queue(topic_name=complete_topic_name, queue_id=queue_id,
                                                   params=params, message_codecs=message_codecs)
                    else:
                        self.init_static_queue(topic_name=topic_name, queue_id=queue_id,
                                               params=params, message_codecs=message_codecs)
                elif params.dynamic_queue_service:
                    service_name = rospy.names.canonicalize_name(params.dynamic_queue_service)
                    update_topic_name = rospy.names.canonicalize_name(params.dynamic_update_topic)
                    rospy.loginfo(f"Initialized dynamic queue ID {queue_id} for {service_name}")
                    try:
                        self.dynamic_queues[service_name] = DynamicQueue(message_codec_id=queue_id,
                                                                         dynamic_message_service=service_name,
                                                                         dynamic_update_topic=update_topic_name,
                                                                         dest_address=params.destination,
                                                                         message_codec=message_codecs[params.codec_id], )

                        self.dynamic_get_next_message_services[service_name] = rospy.ServiceProxy(service_name,
                                                                                                  GetNextQueuedMessage,
                                                                                                  persistent=False)
                        self.dynamic_queue_update_publishers[service_name] = rospy.Publisher(
                            update_topic_name, UpdateQueuedMessage, queue_size=10)

                    except Exception as e:
                        rospy.logerr(f"Error setting up dynamic queue for {service_name}: {e}")

            # QUESTION(lindzey): It looks like the packet codec is never used?
            #   => AAAAH. This is called for its side effects. Maybe rename to make more clear?
            # The packet codec initializer sets the packet codec on each message codec
            packet_codec = packet_codec_class.PacketCodec(src=self.src, **packet_codec_details)
            rospy.loginfo("Added packet codec with {} message codecs".format(len(message_codecs)))

    def init_static_queue(self, topic_name: str, queue_id: int, params: QueueParams, message_codecs) -> None:
        new_queue = MessageQueue(master_message_queue=self._queued_messages,
                                 dest_address=params.destination,
                                 priority=params.priority,
                                 message_codec_id=queue_id,
                                 topic=topic_name,
                                 order=params.queue_order,
                                 maxsize=params.queue_maxsize,
                                 # QUESTION: Why does the queue own the codec?
                                 message_codec=message_codecs[queue_id],
                                 )
        self.topic_queue[topic_name] = new_queue
        rospy.Subscriber(topic_name,
                         AnyMsg,
                         partial(self.on_incoming_message, topic_name=topic_name))

        if params.is_active:
            self.active_queue_names.add(topic_name)

        rospy.loginfo(f"Initialized static queue ID {queue_id} for {topic_name}")

    def publish_queue_status(self) -> None:
        highest_priority = -1
        queued_dest_addresses = set()
        message_counts = {}

        for key, group in groupby(self._queued_messages, lambda x: (x.priority, x.dest)):
            group_count = sum(1 for _ in group)
            message_counts[key] = group_count
            highest_priority = max(highest_priority, key[0])
            queued_dest_addresses.add(key[1])

        total_message_count = len(self._queued_messages)

        # Build the QueueStatus message
        hdr = Header(stamp=rospy.get_rostime())
        summary_message_counts = []
        for k,v in message_counts.items():
            summary = SummaryMessageCount(priority=k[0], dest_address=k[1], message_count=v)
            summary_message_counts.append(summary)
        queue_summaries = []
        for topic, queue in self.topic_queue.items():
            try:
                next_message_size_bits = queue._queue[0].length_bits
            except IndexError:
                next_message_size_bits = 0
            total_bits_in_queue = sum(q.length_bits for q in queue._queue)
            queue_summary = QueueSummary(queue_id=queue.message_codec_id,
                                         topic=topic,
                                         priority=queue.priority,
                                         enabled=(topic in self.active_queue_names),
                                         message_count=len(queue._queue),
                                         next_message_size_bits=next_message_size_bits,
                                         total_bits_in_queue=total_bits_in_queue)
            queue_summaries.append(queue_summary)

        msg = QueueStatus(header=hdr,
                          total_message_count=total_message_count,
                          highest_priority=highest_priority,
                          queued_dest_addresses=queued_dest_addresses,
                          summary_message_counts=summary_message_counts,
                          queue_summaries=queue_summaries)
        self.queue_status_pub.publish(msg)

    def on_incoming_message(self, msg_data, topic_name):
        # type: (AnyMsg, str) -> None
        connection_header = msg_data._connection_header['type'].split('/')
        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        #print 'Message type detected as ' + msg_type

        try:  # to get ROSMSG class
            msg_class = getattr(import_module(ros_pkg), msg_type)
        except AttributeError:  # if not found, try to get ROSSRV class messages (Request/Response)
            msg_class = getattr(import_module(f'{connection_header[0]}.srv'), msg_type)

        msg = msg_class()
        msg.deserialize(msg_data._buff)

        rospy.logdebug("On incoming message: {} ({})".format(topic_name, msg_type))

        # Check if incoming msg topic name matches any of the active queue topic names
        if topic_name in self.active_queue_names:
            rospy.logdebug("TOPIC: {} in active queue list, appending msg".format(topic_name))
            self.topic_queue[topic_name].append(msg)
            # NOTE(lindzey): Don't publish status messages here, because it
            #    will lead to maxing out CPU thanks to the subsea queue_node
            #    subscribing to its own status messages.
        else:
            rospy.logdebug("TOPIC: {} not in active queue list, skipping msg".format(topic_name))

    def mark_transmitted(self, msg: QueuedMessage) -> None:
        # For now, we don't do this for locally queued messages (since we don't track it properly)
        #msg.queue.mark_transmitted(msg)

        # ... but we do track it for dynamic queues
        if isinstance(msg.queue, DynamicQueue):
            if msg.queue.dynamic_message_service in self.dynamic_queue_update_publishers:
                update_msg = UpdateQueuedMessage(header=Header(stamp=rospy.Time.now()),
                                                 message_id=msg.message_id_in_queue,
                                                 event=UpdateQueuedMessage.EVENT_LINK_TRANSMITTED)
                self.dynamic_queue_update_publishers[msg.dynamic_queue_name].publish(update_msg)
                rospy.logdebug(f"Marked message as transmitted: {msg.message_id_in_queue} on {msg.queue.dynamic_message_service}")

    def handle_aquire_lock_failure(self):
        # Helper method called when a handle_get_next_packet() call is triggered while another handle_get_next_packet() is still in progress.
        # ..  
        self.acquire_lock_attempt_count += 1
        if self.acquire_lock_attempt_count > self.acquire_lock_max_attempts:
            # This is a brutish way to try to avoid an error condition where we get "stuck" servicing one request
            rospy.logfatal(
                f"FATAL: Get next packet data failed {self.acquire_lock_max_attempts} times. IF THIS NODE IS NOT CONFIGURED TO RESPAWN, THIS NODE WILL NOT RESPAWN BY DEFAULT.")
            # cannot just call sys.exit from a child thread... We want roslaunch to respawn us. 
            # .. When we kill this node by it's PID, roslaunch will respawn it (if it's set to). TODO: improve error reporting
            os.kill(os.getpid(), signal.SIGINT)
            return True
        return False

    def handle_get_next_packet(self, req: GetNextPacketDataRequest) -> GetNextPacketDataResponse:
        # Things get squirrely if there is more than one request at the same time... our behavior isn't defined.
        # If there is still a request in progress when we get another one, we want to later request to return nothing
        have_lock = self.get_next_packet_data_lock.acquire(blocking=False)
        if not have_lock:
            # if we don't have the lock, we count this attempt and return. 
            # .. only the thread with the lock should release it. Intentionally left out of the try/catch
            self.handle_aquire_lock_failure()
            return GetNextPacketDataResponse()
        
        try:
            # we got the lock, now reset our counter
            self.acquire_lock_attempt_count = 0

            # before calling get_next_packet, set default tag_config. 
            # .. will be ovewritten with settings for this packet during get_next_packet
            # .. this just makes sure we have a default value before the start of every packet creation
            self.current_packet_tag_config = TagConfig()

            if not req.minimum_priority:
                # a priority of 0 is equivalent to None.
                req.minimum_priority = None
            if not req.match_dest:
                # if req.dest is not filled in by requester, it's 0, we actually want None in this case
                req.dest = None

            # this method handles finding the highest priority packet codec, then using the packet codec type to encode the packet returned.
            # TODO: Need to validate that selected codec can fit with overhead
            # TODO: Support multiple packet_codecs in one packet
            response, messages_in_packet = self.get_next_packet(req=req)
            # planning to do more here with status on outbound traffic
            # ..
            return response
        except:
            rospy.logerr(f"Exception in get_next_packet_data: \nTraceback:\n{traceback.format_exc()}")
            return GetNextPacketDataResponse()
        finally:
            # we ONLY call release if WE have the lock. Once enter try/catch he can be sure we have the lock.
            self.get_next_packet_data_lock.release()

    def get_next_packet(self, req: GetNextPacketDataRequest):
        # assumes req.minimum_priority is linted by caller (handle_get_next_packet)
        # this is the default value. Tags may change the minimum_priority in the future
        minimum_priority = req.minimum_priority
        messages_in_packet = []
        
        try:
            # Setup the link_layer TagConfig for this packet:
            #   - the tag_config instance is instantiated with the require_tags and exclude_tags from the request
            #   - tag_config has a method that will generate the "final/filtered" exclude_queue_ids list
            #       - *message_queue generates the lists of queue_ids that fall into the tagged categories for this packet,
            #           .. with that info and a list of all the queue_ids, tag_config can generate the exclude_queue_ids for this packet.
            tag_config: TagConfig = TagConfig(require_tags=req.require_tags, exclude_tags=req.exclude_tags, minimum_priority=minimum_priority)
            tag_config.set_exclude_list_from_tags(queue_params=self.queue_params)
            if tag_config.exclude_queue_ids: 
                rospy.loginfo_throttle(1, 
                    f'DEBUG: link layer (static) queues that did not meet TX criteria: exclude_queue_ids: {tag_config.exclude_queue_ids}, minimum_priority: {minimum_priority}')
            # now that we have made the tag_config for THIS packet, set the class attr
            self.current_packet_tag_config = tag_config
        except:
            # if there is an issue with the tag_config for this request, print error and don't use tags for this packet
            rospy.logerr_throttle(1, f'ERROR: Problem creating TagConfig... {traceback.format_exc()}')
        else:
            rospy.logdebug_throttle(1, f'DEBUG: Created TagConfig for this req. tag_config: {tag_config}')

        rospy.logdebug("DEBUG: Getting next packet data for transmission: " + str(req).replace('\n', ', '))
        try:
            # since we have forced single-thread operation here,
            # .. we can track which dynamic message queues have already been serviced
            self.current_packet_dynamic_queue_cache = {}

            num_miniframe_bytes = req.num_miniframe_bytes
            num_dataframe_bytes = req.num_dataframe_bytes
            available_packet_bytes = num_miniframe_bytes + num_dataframe_bytes

            # this is how we get the packet codec type of the registered queue with msgs and the highest priority.
            # .. this currently means we will use this packet codec type to fill the rest of the packet
            packet_codec, dest_address = self.get_highest_priority_codec(available_packet_bytes=available_packet_bytes,
                                                                         dest_address=req.dest if req.match_dest else None,
                                                                         minimum_priority=minimum_priority)

            if not packet_codec:
                rospy.logdebug_throttle(1, 
                    f"No Highest priority packet_codec found that matched criteria:\navailable space in packet: {available_packet_bytes} bytes (UNUSED)\n---\nminimum_priority: {minimum_priority}\ntag_config: {tag_config}")
                return GetNextPacketDataResponse(), messages_in_packet

            rospy.logdebug_throttle(1, 
                f"Highest priority packet_codec found.\npacket_codec overhead: {packet_codec.codec_overhead_bits * 8} bytes, available space in packet: {available_packet_bytes} bytes\n---\nminimum_priority: {minimum_priority}\ntag_config: {tag_config}")

            # the packet_codec.encode_payload method calls self.get_next_message in a loop with the available space shrinking until the packet is full
            # TODO: check for a race condition where the dest of the top message in the queue changes
            messages_in_packet, miniframe_bytes, dataframe_bytes = packet_codec.encode_payload(num_miniframe_bytes,
                                                                                               num_dataframe_bytes,
                                                                                               self)

            for msg in messages_in_packet:
                self.mark_transmitted(msg)

        except:
            rospy.logerr(f"Exception in get_next_packet_data() \nTraceback:\n{traceback.format_exc()}")
            return GetNextPacketDataResponse(), messages_in_packet
        else:
            # handle_get_next_packet was a success! We are returning an encoded payload
            return GetNextPacketDataResponse(dest=dest_address,
                                             miniframe_bytes=miniframe_bytes,
                                             dataframe_bytes=dataframe_bytes,
                                             queued_message_ids=[],
                                             num_miniframe_bytes=len(miniframe_bytes),
                                             num_dataframe_bytes=len(dataframe_bytes),
                                             num_messages=len(messages_in_packet)), messages_in_packet

    def get_highest_priority_codec(self, available_packet_bytes: int,
                                         dest_address: Optional[int] = None, 
                                         minimum_priority: int = None):
        highest_priority_message = self.get_next_message(max_size_in_bits=available_packet_bytes*8,
                                                         include_packet_overhead_in_size=True,
                                                         dest_address=dest_address,
                                                         minimum_priority=minimum_priority,
                                                         )

        if highest_priority_message:
            packet_codec = highest_priority_message.message_codec.packet_codec
            dest_address = highest_priority_message.dest
        else:
            packet_codec = None
            
        return packet_codec, dest_address

    def get_next_message(self, max_size_in_bits: int = 131072,
                         include_packet_overhead_in_size = False,
                         dest_address: Optional[int] = None,
                         packet_codec: Optional[str] = None,
                         check_reachability: bool = True,
                         minimum_priority: int = None,
                         exclude_messages: Optional[List[QueuedMessage]] = []) -> Optional[QueuedMessage]:

        # We need these "link_layer" tags to pass to dynamic queue endpoints.
        # These other nodes may have nested tags and exclusion criteria, this is how we tell those nodes about,
        # .. the link_layer tags. Now nodes using the dynamic queue service can customize their behavior
        link_layer_require_tags = self.current_packet_tag_config.require_tags
        link_layer_exclude_tags = self.current_packet_tag_config.exclude_tags
        exclude_queue_ids = self.current_packet_tag_config.exclude_queue_ids

        # Query registered dynamic queues
        #   - we need to find the highest priority dynamic queue that can fit in the available packet space.
        #   - we have a cache for responses to our dynamic_queue queries MADE DURING THIS PACKET. (self.current_packet_dynamic_queue_cache)
        #   - the cache will hit every dynamic queue once on the query_dynamic_queues() call below.
        #   - we actually build the packet in later queries of the queue (e.g., packet_codec.encode_payload).
        #   - if we get a highest_priority_dynamic_queue, we take the GetNextQueuedMessageResponse and use it to make the QueuedMessage.
        try:
            highest_priority_dynamic_queue, highest_dynamic_priority = self.query_dynamic_queues(
                                                            max_size_in_bits=max_size_in_bits,
                                                            include_packet_overhead_in_size=include_packet_overhead_in_size,
                                                            dest_address=dest_address,
                                                            packet_codec=packet_codec,
                                                            check_reachability=check_reachability,
                                                            minimum_priority=minimum_priority,
                                                            exclude_messages=exclude_messages,
                                                            link_layer_require_tags=link_layer_require_tags,
                                                            link_layer_exclude_tags=link_layer_exclude_tags,
                                                        )
        except:
            rospy.logerr(f"get_next_message Error querying dynamic queues:\n{traceback.format_exc()}")
            highest_priority_dynamic_queue, highest_dynamic_priority = None, None

        # Query the static queues
        #   - all the static queues are managed by us, (message queue node) so it simplifies things (e.g., we don't need a cache for responses)
        #   - we don't pass the tags to the static queue query because:
        #       - we have already done the filtering for the link layer. exclude_queue_ids has all the info static queues need to "use tags"
        highest_priority_static_message = self.query_static_queues(
                                                            max_size_in_bits=max_size_in_bits,
                                                            include_packet_overhead_in_size=include_packet_overhead_in_size,
                                                            dest_address=dest_address,
                                                            packet_codec=packet_codec,
                                                            check_reachability=check_reachability,
                                                            minimum_priority=minimum_priority,
                                                            exclude_queue_ids=exclude_queue_ids,
                                                            exclude_messages=exclude_messages,
                                                        )
        if not highest_priority_static_message and not highest_dynamic_priority:
            # No message in static queues OR dynamic_queues
            return None
        if highest_priority_static_message and not highest_dynamic_priority:
            # No messages in dynamic queues, BUT static queues have a message
            message = highest_priority_static_message

        elif highest_dynamic_priority and not highest_priority_static_message:
            # Dynamic queues have a message, but static don't
            message = self.get_message_from_queue_response(highest_priority_dynamic_queue, self.current_packet_dynamic_queue_cache[highest_priority_dynamic_queue])
            # Clear the dynamic_queue query response (for this queue) from the cache so we can check this queue if/when we re-enter get_next_message(), this packet.
            del self.current_packet_dynamic_queue_cache[highest_priority_dynamic_queue]

        elif highest_dynamic_priority > highest_priority_static_message.priority:
            # we have messages in the dynamic queues AND static queues.
            # for now, we use priority to determine which queue type gets to fill this message on THIS call to get_next_message().
            # TODO: get_next_message() is called multiple times throughout building a packet from different contexts. Worth optimizing this?
            message = self.get_message_from_queue_response(highest_priority_dynamic_queue, self.current_packet_dynamic_queue_cache[highest_priority_dynamic_queue])
            # Clear the dynamic_queue query response (for this queue) from the cache so we can check this queue if/when we re-enter get_next_message(), this packet.
            del self.current_packet_dynamic_queue_cache[highest_priority_dynamic_queue]
        else:
            # TODO: Bug, when registered dynamic_queues:
            #   - have higher priority than static_queues AND enough traffic to fill the packet, 
            #     .. you could be in a scenario where the static queues are never emptied.
            #   - if the priority is the same, we defer to static queues
            message = highest_priority_static_message

        return message

    def verify_dynamic_queue_response(self,
                                      tag_config: TagConfig,
                                      response: GetNextQueuedMessageResponse,
                                      max_size_in_bits: int = 131072) -> bool:
        if not response.has_message:
            return False

        if response.data_size_in_bits > max_size_in_bits:
            rospy.logerr(f"ERROR: Queue response from {queue} has size {response.data_size_in_bits}, larger than max {max_size_in_bits}.  Skipping.")
            return False

        if tag_config.minimum_priority is not None:
            # check if this response priority meet min
            if response.priority < tag_config.minimum_priority:
                rospy.logdebug_throttle(1, f"DEBUG: dynamic_queue: response.priority is < minimum_priority({tag_config.minimum_priority}). response.priority: ({response.priority})")
                return False

        # check that tags in dynamic_queue response follow link_layer tag set logic
        if not tag_config.is_valid_tag_set(tags=response.msg_tags):
            rospy.logerr(f"ERROR: dynamic_queue: msg_tags in response do NOT follow tag set logic!\nmsg_tags:{response.msg_tags}\nrequire_tags:{tag_config.require_tags}\nexclude_tags:{tag_config.exclude_tags}")
            return False

        rospy.logdebug_throttle(1, f"DEBUG: dynamic_queue: valid response! tags in response follow tag set logic and other criteria: response:\n{response}")
        return True

    def query_dynamic_queues(self, 
                             max_size_in_bits: int = 131072,
                             include_packet_overhead_in_size = False,
                             dest_address: Optional[int] = None,
                             packet_codec: Optional[str] = None,
                             check_reachability: bool = True,
                             minimum_priority: int = None,
                             exclude_messages: Optional[List[QueuedMessage]] = None,
                             link_layer_require_tags=None,
                             link_layer_exclude_tags=None) -> tuple[Optional['DynamicQueue'], Optional[int]]:
        """
        Query the registered dynamic queues using the specified message filter criteria.
        Returns the highest priority dynamic queue matching those criteria.

        This uses and populates the dynamic queue cache.  The cache "lives" for the duration of a single packet.  The
        cache object locking is handled by the global lock in get_net_packet_data.

        :param max_size_in_bits: The maximum size of the message to retrieve from the dynamic queues, in bits.
        :param include_packet_overhead_in_size: Whether to include packet codec overhead in the message size
            calculation. This is used to get the packet codec for a new packet.  Defaults to False.
        :param dest_address: If specified, only return messages with this destination address. Defaults to None.
        :param packet_codec: The packet codec to use for the message retrieval. Defaults to None, not currently used.
        :param check_reachability: Deprecated.  Whether to check if the destination address is reachable. Defaults to
            True, but is ignored (and deprecated)
        :param minimum_priority: The minimum priority of the message to retrieve. Defaults to None.
        :param exclude_messages: A list of QueuedMessage objects that should be excluded from the retrieval (used to
            avoid including duplicates of the same message in a single packet). Defaults to None.
        :param link_layer_require_tags: A list of tags that the message must have. Defaults to None.
        :param link_layer_exclude_tags: A list of tags that the message must not have. Defaults to None.
        :return: A tuple containing the highest priority dynamic queue and its priority. If no messages are found in
            the dynamic queues, returns (None, None).

        """
        if link_layer_exclude_tags is None:
            link_layer_exclude_tags = []
        if link_layer_require_tags is None:
            link_layer_require_tags = []
        if exclude_messages is None:
            exclude_messages = []

        tag_config = TagConfig(require_tags=link_layer_require_tags, 
                               exclude_tags=link_layer_exclude_tags, 
                               minimum_priority=minimum_priority)

        start_time = time.time()  # for profiling
        for dynamic_queue in self.dynamic_queues.values():
            dynamic_queue_name = dynamic_queue.dynamic_message_service

            message_max_size_in_bits = max_size_in_bits
            try:
                packet_codec_overhead_bits = dynamic_queue.message_codec.packet_codec.codec_overhead_bits
                if include_packet_overhead_in_size:
                    message_max_size_in_bits = max_size_in_bits - packet_codec_overhead_bits
            except:
                rospy.logerr_throttle(1, "Error getting packet codec overhead for dynamic queue")

            # Check to see if we already have a cached response from the last time we did this.
            cached_response = self.current_packet_dynamic_queue_cache.get(dynamic_queue_name)
            if cached_response:
                # Check to see if the cached response matches our current query
                # There are some conditions that aren't covered here... we assume that the max_size_in_bits only gets
                # smaller over the course of a packet, and that we only ever exclude more messages, for example.
                # We also don't check to see if required or excluded tags have changed, since they stay the same for
                # a packet.
                if not cached_response.has_message:
                    # don't query queues with no messages more than once, go to the next queue
                    continue
                # Now, check parameters we might care about
                if cached_response.data_size_in_bits < message_max_size_in_bits and (
                        cached_response.message_id not in exclude_messages):
                    # the data is not larger than the available space
                    # AND the message_id of this message is not on the exclude list
                    if dest_address is not None:
                        # the request had match_dest:True, so we need to check that the dest address in the cached
                        # response matches it.
                        if cached_response.dest_address == dest_address:
                            # Use the cache
                            continue
                        else:
                            # Don't use the cache, query again.
                            pass
                    else:
                        continue
                else:
                    # We don't have enough space for the cached response or the message_id in the cache was
                    # on the exclude list.
                    # If we get here, we aren't using the cached response and are going to query again.
                    pass
                # TODO: This doesn't correctly check that the cached packet codec value matches

            # Starting here, we query the dynamic queue (and load/reload the cache)
            # Figure out which messages to exclude (because they are already queued)
            this_queue_exclusion_ids = [m.message_id_in_queue for m in exclude_messages if
                                        m.dynamic_queue_name == dynamic_queue_name]

            # Get a reference to the service
            dynamic_get_next_queued_message = self.dynamic_get_next_message_services[dynamic_queue_name]

            try:
                queue_response = dynamic_get_next_queued_message(max_size_in_bits=message_max_size_in_bits,
                                                                 minimum_priority=minimum_priority,
                                                                 dest_address=dest_address,
                                                                 packet_codec=(
                                                                     str(packet_codec) if packet_codec else ''),
                                                                 exclude_message_ids=this_queue_exclusion_ids,
                                                                 require_tags=link_layer_require_tags,
                                                                 exclude_tags=link_layer_exclude_tags)
                rospy.logdebug(f"Queried {dynamic_queue_name} for {message_max_size_in_bits} bits, got: {queue_response}")
            except rospy.service.ServiceException as e:
                rospy.logerr_throttle(1, f"Error calling get_next_message service on {dynamic_queue_name}: {e}")
                # if we have an error, don't try again for this packet. We act as though this dynamic queue had no
                # messages for this rest of THIS packet
                queue_response = GetNextQueuedMessageResponse(has_message=False)

            # Cache the response
            self.current_packet_dynamic_queue_cache[dynamic_queue_name] = queue_response

        rospy.logdebug(f"Dynamic queue query took {time.time()-start_time} s")

        # Now, figure out what the highest priority dynamic message is, if any,
        #  and handle gating if minimum_priority is passed
        priorities = {}
        for queue, response in self.current_packet_dynamic_queue_cache.items():
            if self.verify_dynamic_queue_response(tag_config=tag_config, response=response, max_size_in_bits=max_size_in_bits):
                priorities[queue] = response.priority

        if len(priorities) > 0:
            sorted_priorities = sorted(priorities.items(), key=lambda x:x[1])
            highest_priority_queue = sorted_priorities[-1][0]
            highest_priority = sorted_priorities[-1][1]
            return highest_priority_queue, highest_priority
        # No messages in dynamic queues
        return None, None

    def query_static_queues(self, 
                            max_size_in_bits: int = 131072,
                            include_packet_overhead_in_size = False,
                            dest_address: Optional[int] = None,
                            packet_codec: Optional[str] = None,
                            check_reachability: bool = True,
                            minimum_priority: int = None,
                            exclude_queue_ids: Optional[List[int]] = None,
                            exclude_messages: Optional[List[QueuedMessage]] = None) -> Optional[QueuedMessage]:
        # TODO: exclude_messages is not currently used for static queues (but it should be).  This functionality is
        #   handled by automatically marking any static queue message as transmitted whenever we add it to a packet
        #   inside the packet codec by setting get_next_payload_smaller_than_bits(remove_from_queue=True)
        if exclude_messages is None:
            exclude_messages = []
        if exclude_queue_ids is None:
            exclude_queue_ids = []
        next_message = None

        for message in sorted(self._queued_messages, key=lambda m: m.priority, reverse=True):
            if message.queue.message_codec_id in exclude_queue_ids:
                rospy.logdebug_throttle(1, f"Disqualified: queue_id: {message.queue.message_codec_id} in exclude_queue_ids list")
                continue
            if message.queue.subscribe_topic not in self.active_queue_names:
                rospy.logdebug_throttle(1, f"Disqualified: inactive queue")
                continue
            if check_reachability and not self.destination_reachable[message.dest]:
                rospy.logdebug_throttle(1, f"Disqualified: Reachability")
                continue
            if dest_address and (dest_address != message.dest):
                # Move on to check the next message
                rospy.logdebug_throttle(1, f"Disqualified: dest_address")
                continue
            if packet_codec and (packet_codec != message.message_codec.packet_codec):
                # Move on to check the next message
                rospy.logdebug_throttle(1, f"Disqualified: packet_codec")
                continue
            if include_packet_overhead_in_size:
                if (message.length_bits + message.message_codec.packet_codec.codec_overhead_bits) > max_size_in_bits:
                    rospy.logdebug_throttle(1, f"Disqualified: message too large (with codec overhead)")
            else:
                if message.length_bits > max_size_in_bits:
                    rospy.logdebug_throttle(1, f"Disqualified: message too large")
                    continue
            if minimum_priority is not None and message.priority < minimum_priority:
                rospy.logdebug_throttle(1, f"Disqualified: message.priority is < minimum_priority({minimum_priority}). message.priority: ({message.priority})")
                continue
            # if we aren't disqualified by now, this is our message.
            next_message = message
            break

        return next_message

    def get_message_from_queue_response(self, dynamic_queue, queue_response: GetNextQueuedMessageResponse) -> QueuedMessage:
        # this is only called for dynamic_queues. This allows tracking info to flow back to the dynamic queue client node
        message_bits = BitArray(bytes=queue_response.data, length=queue_response.data_size_in_bits)

        next_message = QueuedMessage(queue=self.dynamic_queues[dynamic_queue],
                                     message=None,
                                     message_codec=self.dynamic_queues[dynamic_queue].message_codec,
                                     dest=120,
                                     time_added=datetime.utcnow(),
                                     dynamic_queue_name=self.dynamic_queues[dynamic_queue].dynamic_message_service,
                                     message_id_in_queue=queue_response.message_id,
                                     payload_bits=message_bits,
                                     length_bits=len(message_bits))
        return next_message

    def on_neighbor_reachability(self, msg: NeighborReachability) -> None:
        self.destination_reachable[msg.dest] = msg.reachable

    def handle_queue_priority(self, msg: QueuePriority) -> None:
        try:
            sub_topic = self.queue_params[msg.queue_id].subscribe_topic
        except KeyError:
            rospy.logerr("Can't enable unrecognized Queue ID: {}".format(msg.queue_id))
            return
        self.topic_queue[sub_topic].priority_update(msg.priority)

    def handle_queue_enable(self, msg: QueueEnable) -> None:
        try:
            sub_topic = self.queue_params[msg.queue_id].subscribe_topic
        except KeyError:
            rospy.logerr("Can't enable unrecognized Queue ID: {}".format(msg.queue_id))
            return
        if msg.enable:
            self.active_queue_names.add(sub_topic)
        else:
            try:
                self.active_queue_names.remove(sub_topic)
            except KeyError:
                rospy.loginfo("Cannot disable currently-inactive queue ID {} ({})"
                              .format(msg.queue_id, sub_topic))

    def spin(self) -> None:
        # Issue status periodically.
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.publish_queue_status()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = MessageQueueNode()
        rospy.loginfo("Message Queue shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("Message Queue shutdown (interrupt)")
