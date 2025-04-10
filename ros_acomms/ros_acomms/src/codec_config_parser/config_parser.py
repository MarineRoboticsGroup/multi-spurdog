from dataclasses import dataclass
from ltcodecs import RosMessageCodec
import roslib
import rospy
import yaml
from rospy_yaml_include.yaml_include import RospyYamlInclude
from typing import Dict, NewType, Optional, List
import acomms_codecs

QueueId = NewType('QueueId', int)
MessageCodecId = NewType('MessageCodecId', int)

@dataclass
class QueueParams:
    msg_class: type
    dynamic_queue_service: Optional[str]
    dynamic_update_topic: Optional[str]
    subscribe_topic: Optional[str]
    publish_topic: Optional[str]
    codec_id: MessageCodecId
    queue_order: str  # TODO: I'd like this to be an enum corresponding to 'fifo', 'lifo'
    queue_maxsize: int
    priority: Optional[int]
    is_active: bool
    destination: Optional[int]
    tags: Optional[List[str]]
    params_dict: Dict


def load_custom_codecs() -> None:
    """
    Helper function to load custom codecs using a recursively-searched rosparam ("custom_packet_codec_paths")

    The custom codecs are loaded in-place in the acomms_codecs.packet_codecs dictionary
    """
    # Load custom codecs.  We do this here because we can't really get rosparams until we have a node.
    custom_packet_codec_paths_param_name = rospy.search_param('custom_packet_codec_paths')
    if custom_packet_codec_paths_param_name:
        custom_packet_codec_paths = rospy.get_param(custom_packet_codec_paths_param_name)
        acomms_codecs.load_custom_codecs(custom_packet_codec_paths)

    rospy.loginfo(f"Loaded Packet Codecs: {acomms_codecs.packet_codecs}")

def read_packet_codec_yaml() -> dict:
    """
    Read the packet codec file from the path specified in the "packet_codec_file" rosparam and load it into a dictionary.

    This also loads the custom packet codecs

    This must be run from within a ROS node.  It will search up the namespace tree for the "packet_codec_file" and "codec_directory" params.
    :return: Dictionary containing the packet codecs from the YAML file.
    """
    packet_codec_file_param_name = rospy.search_param('packet_codec_file')
    packet_codec_filename = rospy.get_param(packet_codec_file_param_name)
    codec_directory_param_name = rospy.search_param('codec_directory')
    if codec_directory_param_name:
        codec_dir = rospy.get_param(codec_directory_param_name, None)
    else:
        codec_dir = None
    try:
        with open(packet_codec_filename, 'r') as yaml_file:
            constructor = RospyYamlInclude(base_directory=codec_dir)
            packet_codecs = yaml.load(yaml_file, Loader=constructor.add_constructor())
    except Exception as e:
        rospy.logfatal(
            f"Fatal error reading message codec config from {packet_codec_filename} (with codec directory {codec_dir}): {e}")
        raise e

    return packet_codecs

def get_queue_params(packet_codec_param: Dict) -> Dict[int, QueueParams]:
    # Queues are indexed by their ID because that's what goes in the header
    # to identify the appropriate publication parameters for packet dispatch
    queue_params: Dict[QueueId, QueueParams] = {}
    config_version = packet_codec_param.get('config_version')
    if config_version is None:
        # Use legacy, where we have a one-to-one mapping between queues and message codecs
        for message_codec_param in packet_codec_param['message_codecs']:
            both_ids = message_codec_param["id"]
            ros_type_name = message_codec_param['ros_type']
            msg_class = roslib.message.get_message_class(ros_type_name)

            if msg_class is None:  # was not ROSMSG instance, try ROSSRV
                msg_class = roslib.message.get_service_class(ros_type_name)

            if not msg_class:
                raise TypeError('Invalid ROS type "{}" for codec ID {}'
                                .format(ros_type_name, both_ids))
            if both_ids in queue_params.keys():
                rospy.logerr(f"Duplicate id {both_ids} in codec config.  Only the last entry will be used.")

            queue_params[both_ids] = QueueParams(msg_class,
                                                 message_codec_param.get('dynamic_queue_service', None),
                                                 message_codec_param.get('dynamic_update_topic', None),
                                                 message_codec_param.get("subscribe_topic", None),
                                                 message_codec_param.get("publish_topic", None),
                                                 both_ids,
                                                 message_codec_param.get("queue_order", 'fifo'),
                                                 message_codec_param.get("queue_maxsize", 10),
                                                 # if not specified, MessageQueueNode sets a default priority
                                                 message_codec_param.get("priority", None),
                                                 message_codec_param.get("is_active", True),
                                                 # if not specified, MessageQueueNode sets a default destination
                                                 message_codec_param.get("default_dest", None),
                                                 # optional list of tags to associate with this queue
                                                 message_codec_param.get("tags", []),
                                                 # Capture all values that are set, so that other users of the codec
                                                 # config parser can add arbitrary entries to the param dictionary
                                                 params_dict=message_codec_param,
                                                 )
    else:
        # Use new version that separates the queue and codec definitions.
        raise Exception("NYI")
    return queue_params


def get_message_codecs(packet_codec_param):
    message_codecs: Dict[MessageCodecId, QueueParams] = {}
    config_version = packet_codec_param.get('config_version')
    if config_version is None:
        # Use legacy, where we have a one-to-one mapping between queues and message codecs
        for message_codec_param in packet_codec_param['message_codecs']:
            # NOTE(lindzey): The following TODOs were copied over from packet_dispatch_node
            # TODO: allow custom codecs
            # like this: MyClass = getattr(importlib.import_module("module.submodule"), "MyClass")
            # TODO: make this work with orthogonal packet codecs
            both_ids = message_codec_param["id"]
            ros_type_name = message_codec_param['ros_type']
            msg_class = roslib.message.get_message_class(ros_type_name)

            if msg_class is None:  # was not ROSMSG instance, try ROSSRV
                msg_class = roslib.message.get_service_class(ros_type_name)

            if not msg_class:
                raise TypeError('Invalid ROS type "{}" for codec ID {}'
                                .format(ros_type_name, both_ids))
            # If there is no fields dictionary, allow the codec to determine the fields through introspection.
            fields = message_codec_param.get('fields', None)
            message_codecs[both_ids] = RosMessageCodec(ros_type=msg_class, fields_dict=fields)
    else:
        # Use new version that separates the queue and codec definitions.
        raise Exception("NYI")
    return message_codecs
