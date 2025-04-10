#!/usr/bin/env python3

import yaml, json, re
import numpy as np
import itertools
from rospy_yaml_include.yaml_include import RospyYamlInclude
from pathlib import Path
import traceback
from collections import namedtuple, deque
import rospy
import roslib.message
from rospy import AnyMsg
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from codec_config_parser import QueueParams
from typing import Dict, NewType, Optional, List

FDPMaxBytes4Rate = namedtuple('FDPMaxBytes4Rate', 'rate max_bytes default packet_duration_ms')
ManualTransmitRequest = namedtuple('ManualTransmitRequest', 'subscriber_queue publisher publish_topic subscribe_topic')
ManagedMacNodeEntry = namedtuple('ManagedMacNodeEntry', 'ns is_default set_software_mute select_callback')
re_slot_range = r"^(?P<start>[0-9]*)(:|;|-)(?P<end>[0-9]*)(:|;|-)?((?P<step>[0-9]*))?$"

def deserialze_anymsg(msg_data: AnyMsg):
    topic_type = msg_data._connection_header['type']
    topic_class = roslib.message.get_message_class(topic_type)
    msg = topic_class()
    msg.deserialize(msg_data._buff)
    return msg

def parse_slot_tags(num_slots, slot_tags_param=None):
    # default_slot_tags = {slot: None for slot in range(num_slots)}
    default_slot_tags = {slot: TagConfig() for slot in range(num_slots)}

    if slot_tags_param is None:
        return default_slot_tags, slot_tags_param
    elif isinstance(slot_tags_param, str):
        slot_tags_dict = parse_rospy_yaml_config(slot_tags_param)
        # rosparam server gives the list of dicts, the yaml has an additional dict wrapper
        # .. this is to match the launch file convention of having a slot_tags top level key
        slot_tags_param = slot_tags_dict['slot_tags']

        if slot_tags_param is None:
            return default_slot_tags, slot_tags_param

    if isinstance(slot_tags_param, list):
        # will be list if parsed as a file OR passed as a yaml rosparam by this point
        slot_tags = {}

        # this loop creates the basic dict that might have slots set as a range rather than
        # .. being broken out into each slot. We expand this dict lower down
        for slot_config in slot_tags_param:
            slot = str(slot_config.get('slots'))
            if slot not in slot_tags:
                slot_tags[slot] = TagConfig()

            slot_tags[slot].require_tags.update(list(slot_config.get('require_tags', [])))
            slot_tags[slot].exclude_tags.update(list(slot_config.get('exclude_tags', [])))
            min_priority = slot_config.get('minimum_priority', None)
            if min_priority is not None:
                slot_tags[slot].minimum_priority = min_priority
    else:
        return default_slot_tags, slot_tags_param

    # we parsed the yaml and now turn any slot defined as a range into it's expanded list
    validated_slot_tags = default_slot_tags
    for slot, tags in slot_tags.items():
        try:
            # try basic case, list of tags for a single slot
            slot = int(slot)
        except ValueError:
            # if this is a range of slots, we need to expand it so we have a key:value for every slot
            match = validate_slot_range(slot_range=slot, last_valid_slot=num_slots - 1)
            if match:
                start, end, step = match
                for i in range(start, end + 1, step):
                    # add current tags for range to exisiting list
                    validated_slot_tags[i].require_tags.update(tags.require_tags)
                    validated_slot_tags[i].exclude_tags.update(tags.exclude_tags)
                    if tags.minimum_priority is not None:
                        # only overwrite the minimum_priority if set for this tagged slot
                        validated_slot_tags[i].minimum_priority = tags.minimum_priority
            else:
                # check if "slot" is a list of ints and not a range
                try:
                    for s in slot.split(','):
                        # add current tags for range to exisiting list
                        s = int(s)
                        validated_slot_tags[s].require_tags.update(tags.require_tags)
                        validated_slot_tags[s].exclude_tags.update(tags.exclude_tags)
                        if tags.minimum_priority is not None:
                            validated_slot_tags[s].minimum_priority = tags.minimum_priority
                except ValueError:
                    # TODO, better handling in this edge case
                    pass 
        else:
            # this key:value is just a single int slot and list of tags
            validated_slot_tags[slot].require_tags.update(tags.require_tags)
            validated_slot_tags[slot].exclude_tags.update(tags.exclude_tags)
            if tags.minimum_priority is not None:
                validated_slot_tags[slot].minimum_priority = tags.minimum_priority

    return validated_slot_tags, slot_tags_param

def validate_slot_range(slot_range, last_valid_slot, first_valid_slot=0, default_step=1):
    match = re.match(re_slot_range, slot_range)
    if match:
        # set defaults (overridden if not empty below)
        start, end, step = first_valid_slot, last_valid_slot, default_step
        # we matched on this pattern. fill empty groups with defaults
        if match.groupdict()['start']:
            start = int(match.groupdict()['start'])
        if match.groupdict()['end']:
            end = int(match.groupdict()['end'])
        if match.groupdict()['step']:
            step = int(match.groupdict()['step'])
        
        return [start, end, step]

def format_range(start, end, step):
    return '{}-{}:{}'.format(start, end, step)
    # return '{}-{}:{}'.format(start, end + step, step)

def parse_range(lst):
    if len(lst) == 1:
        return str(lst[0]), []
    if len(lst) == 2:
        return ','.join(map(str,lst)), []

    step = lst[1] - lst[0]
    for i,x,y in zip(itertools.count(1), lst[1:], lst[2:]):
        if y-x != step:
            if i > 1:
                return format_range(lst[0], lst[i], step), lst[i+1:]
            else:
                return str(lst[0]), lst[1:]
    return format_range(lst[0], lst[-1], step), []

def expanded_slots_as_ranges(slots):
    # https://stackoverflow.com/questions/9847601/convert-list-of-numbers-to-string-ranges
    result = []
    while slots:
        partial, slots = parse_range(slots)
        result.append(partial)
    return ','.join(result)

def parse_rospy_yaml_config(yaml_file_name):
    yaml_file_path = Path(yaml_file_name).expanduser().absolute()
    try:
        with open(yaml_file_path, 'r') as yaml_file:
            constructor = RospyYamlInclude(base_directory=None)
            config_dict = yaml.load(yaml_file, Loader=constructor.add_constructor())
    except Exception:
        rospy.logerr(f'Error reading yaml config!: {traceback.format_exc()}')
        return None
    return config_dict

class TagConfig(object):
    """
    This is the class layer that message_queue (outbound traffic node) interfaces too. 
    The subclass(s) will be used by whatever mac wants to support tags. 
    The API for the node providing get_next_data() (message_queue) will use this super class so tags can be handled generically 
    """
    def __init__(self, require_tags=[], exclude_tags=[], minimum_priority=None):
        super(TagConfig, self).__init__()
        self.require_tags = set(require_tags)
        self.exclude_tags = set(exclude_tags)
        self.minimum_priority = minimum_priority
        self.exclude_queue_ids = []  # This is populated when running generate_exclude_list inside get_next_packet

    def is_valid_tag_set(self, tags: List[str] = None) -> bool:
        if tags is None:
            tags = []

        if self.require_tags:
            # if there are any require_tags, these tags MUST overlap
            if not set(self.require_tags).intersection(set(tags)):
                # passed tags are not valid, non are in the required list!
                return False
        if self.exclude_tags:
            # if there are any exclude_tags, these tags must NOT overlap
            if set(self.exclude_tags).intersection(set(tags)):
                # passed tags are not valid, some tags are in exclude list!
                return False

        # tags are valid:
        #   - they do not overlap with exclude tags (if any)
        #   - if there are any required tags, the tags passed overlap
        return True

    def generate_exclude_list(self, require_ids, exclude_ids, ids):
        # don't exclude anything by default
        exclude_list = []
        # logic for handling require tags:
        #   - remove all queue_ids from exclude_list if in require_ids
        if set(ids).intersection(set(require_ids)):
            exclude_list = list(set(ids).difference(set(require_ids)))

        # logic for handling exclude tags:
        #   - add all queue_ids in the exclude_ids list to exclude_list
        if set(ids).intersection(set(exclude_ids)):
            # turn into a set before adding exclude_ids
            exclude_list = set(exclude_list)
            # adding any ids that have deny tags
            exclude_list.update(set(exclude_ids))
            # make it a list again to match other conditions
            exclude_list = list(exclude_list)

        return exclude_list

    def set_exclude_list_from_tags(self, queue_params: Dict[int, QueueParams] = None) -> None:
        # always set exclude_queue_ids to empty list before filling it
        # this method should only be run once for a packet
        self.exclude_queue_ids = []

        if queue_params is None:
            # TODO: we shouldn't hit this case but if we do we should probs print an error somehow
            return 

        # tagged_queue_ids will map the registered-queues-tags to their category for this packet
        tagged_queue_ids = {
            'require_ids': [], # list of queue_ids with tags in require list
            'exclude_ids': [], # list of queue_ids with tags in exclude list
            'ids': list(queue_params.keys()) # all of the queue_ids (with or w/o tags)
        }

        for queue_id, queue_param in queue_params.items():
            # if tags is an empty list, this queue_id will only be in 'ids'
            if queue_param.tags == []:
                rospy.logdebug_throttle(1, f'Queue ID: {queue_id} has NO tags set!')
                continue

            # check queue tags for require
            if set(self.require_tags).intersection(set(queue_param.tags)):
                # there is an overlap, append this queue_id to the required list
                rospy.logdebug_throttle(1, f'Queue ID: {queue_id} has a tag in the require list!')
                tagged_queue_ids['require_ids'].append(queue_id)

            # check queue tags for exclude
            if set(self.exclude_tags).intersection(set(queue_param.tags)):
                # there is an overlap, append this queue_id to the exclude list
                rospy.logdebug_throttle(1, f'Queue ID: {queue_id} has a tag in the exclude list!')
                tagged_queue_ids['exclude_ids'].append(queue_id)

        # now set the class attr. *We set this to an empty list at the start of this method
        self.exclude_queue_ids = self.generate_exclude_list(require_ids=tagged_queue_ids['require_ids'], 
                                                            exclude_ids=tagged_queue_ids['exclude_ids'], 
                                                            ids=tagged_queue_ids['ids'])

    def __repr__(self):
        if not self.require_tags and not self.exclude_tags and not self.minimum_priority:
            return '[]'
        return json.dumps(
            dict(
                require_tags=list(self.require_tags), 
                exclude_tags=list(self.exclude_tags),
                minimum_priority=self.minimum_priority,
            ),
            indent=4,
            default=str,
        )

class PSK_ROS_FDP_Rates(object):
    FDP_MAX_BYTES_FOR_MINI_RATE = [
        None, FDPMaxBytes4Rate(rate=1, max_bytes=96, default=72, packet_duration_ms=1574), # index 1, rate 1
        None, FDPMaxBytes4Rate(rate=3, max_bytes=96, default=72, packet_duration_ms=755),  # index 3, rate 3
        None, FDPMaxBytes4Rate(rate=5, max_bytes=96, default=72, packet_duration_ms=286),  # index 5, rate 5
        None, None, None, None, None, None, None,
    ]

    FDP_MAX_BYTES_FOR_DATA_RATE = [
        None, FDPMaxBytes4Rate(rate=1, max_bytes=192,  default=192,  packet_duration_ms=3072), # index 1, rate 1
        None, FDPMaxBytes4Rate(rate=3, max_bytes=512,  default=512,  packet_duration_ms=3072), # index 3, rate 3
        None, FDPMaxBytes4Rate(rate=5, max_bytes=2048, default=2048, packet_duration_ms=3000), # index 5, rate 5
        None, None, None, None, None, None, None,
    ]

class ManualTransmitRequestQueue(object):
    """docstring for ManualTransmitRequestQueue"""
    def __init__(self, maxlen=1, topic=None, msg_type=None):
        super(ManualTransmitRequestQueue, self).__init__()
        self.maxlen = maxlen
        self.topic_map = {}
        self.next_manual_transmit = None
        self.manual_transmit_status_publisher = rospy.Publisher('tdma/manual_transmit_status', DiagnosticStatus, queue_size=10, latch=True)
        self.init_pub_sub(topic=topic, msg_type=msg_type)

    def add_manual_transmit_topic(self, topic, msg_type, maxlen=None):
        self.init_pub_sub(topic=topic, msg_type=msg_type, maxlen=maxlen)

    def publish_status(self, on_transmit=False):
        name = 'manual_transmit_queues::on_insert'
        if on_transmit:
            name = 'manual_transmit_queues::on_transmit'
        status = DiagnosticStatus(name=name)

        if self.next_manual_transmit:
            status.message = f'publish_on::{self.next_manual_transmit.publish_topic}'
            status.hardware_id = f'listening_on::{self.next_manual_transmit.subscribe_topic}'
        else:
            status.message = 'nothing in manual transmit queue(s)'

        for topic, record in self.topic_map.items():
            status.values.append(KeyValue(key=topic, value=json.dumps(record.subscriber_queue, default=str)))
        self.manual_transmit_status_publisher.publish(status)

    def publish_next(self):
        if self.next_manual_transmit is None:
            return None

        msg = None
        try:
            msg = self.next_manual_transmit.subscriber_queue.popleft()
            # handle multiple lines of nmea if ganged together by newlines
            lines = msg.data.split('\n')
            # even if there are no newlines, this will publish the single command
            for line in lines:
                if line:
                    self.next_manual_transmit.publisher.publish(data=line)
        except:
            # deque is empty
            self.next_manual_transmit = None
        else:
            self.publish_status(on_transmit=True)

        return msg

    def append(self, msg, topic):
        self.topic_map[self.get_tdma_ns_topic_stem(topic=topic)].subscriber_queue.append(msg)

    def get_tdma_ns_topic(self, topic):
        if topic[0] == '/':
            # absolute topic rather than relative
            return f'tdma{topic}'
        return f'tdma/{topic}'

    def get_tdma_ns_topic_stem(self, topic):
        # TODO: handle absolute/relative topics
        # for now, subclasses should always pass absolute topics this handles tdma base class case which only 
        # .. does the nmea_to_modem in it's relative namespace
        if 'nmea_to_modem' in topic:
            return 'nmea_to_modem'
        return topic[topic.index('tdma/') + 4:]

    def init_pub_sub(self, topic=None, msg_type=None, maxlen=None):
        if topic is None or msg_type is None:
            return
        if maxlen is None:
            maxlen = self.maxlen

        self.topic_map[topic] = ManualTransmitRequest(
                subscriber_queue=deque(maxlen=maxlen),
                publisher=rospy.Publisher(topic, msg_type, queue_size=10),
                publish_topic=topic,
                subscribe_topic=self.get_tdma_ns_topic(topic=topic),
            )
        rospy.Subscriber(self.get_tdma_ns_topic(topic=topic), msg_type, self.on_manual_transmit_request)

    def on_manual_transmit_request(self, msg):
        topic = msg._connection_header['topic']
        self.append(msg=msg, topic=topic)

        # self.next_manual_transmit gets set here initially
        # .. it will switch to a new queue when it's transmitted everything in self.next_manual_transmit
        # .. if a queue is never fully cleared, there will be issues as we don't move to another queue until the active one is empty
        if self.next_manual_transmit is None or not len(self.next_manual_transmit.subscriber_queue):
            self.next_manual_transmit = self.topic_map[self.get_tdma_ns_topic_stem(topic=topic)]

        self.publish_status()

    def handle_active_queue(self):
        return self.publish_next()
