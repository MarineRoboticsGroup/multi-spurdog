#! /usr/bin/python3

from codec_config_parser.config_parser import get_message_codecs, get_queue_params
from typing import Dict, List
from acomms_codecs import packet_codecs
from ltcodecs import RosMessageCodec
import rospy
import argparse
from packet_dispatch_node import DecoderListEntry
import yaml
from rospy_yaml_include.yaml_include import RospyYamlInclude


def check_codec_config_yaml(yaml_file_name, codec_dir):
    with open(yaml_file_name, 'r') as yaml_file:
        constructor = RospyYamlInclude(base_directory=codec_dir)
        packet_codec_param = yaml.load(yaml_file, Loader=constructor.add_constructor())

    decoder_list: List[DecoderListEntry] = []
    for packet_codec in packet_codec_param:
        message_codecs: Dict[int, RosMessageCodec] = get_message_codecs(packet_codec)
        queue_params = get_queue_params(packet_codec)
        # Convert SRC IDs to integers, since yaml dict keys have to be strings
        try:
            src_names: dict = {int(k):v for k,v in packet_codec.get('src_names', {}).items()}
            print(f"Names: {src_names}")
        except Exception as e:
            print(f"Error parsing src_names parameter: {e}")

        pub_names = []

        for queue_id, params in queue_params.items():
            pub_name = rospy.names.canonicalize_name(params.publish_topic)
            codec = message_codecs[queue_id]
            min_size = codec.min_size_bits
            max_size = codec.max_size_bits

            print(f"ID: {queue_id}\t{pub_name}\t{params.msg_class}\t{min_size}\t{max_size}")

            if ("{src}" in params.publish_topic) or ("{name}" in params.publish_topic):
                # In this case, we will have a dict of message publishers that correspond to different SRC IDs
                for src, name in src_names.items():
                    pub_name = params.publish_topic.format(src=src, name=name)
                    pub_name = rospy.names.canonicalize_name(pub_name)
                    pub_names.append(pub_name)
            else:
                pub_name = rospy.names.canonicalize_name(params.publish_topic)
                pub_names.append(pub_name)

        print(f"Publishers:\n{pub_names}")

        packet_codec.pop('message_codecs')
        try:
            packet_codec.pop('src_names')
        except:
            pass

        entry = DecoderListEntry(**packet_codec)
        decoder_list.append(entry)
        rospy.loginfo("Added decoder: {}".format(entry))

        # replace name with instance of packet codec
        # ToDO make this less ugly and stupid
        entry.packet_codec = packet_codecs[entry.packet_codec](message_codecs,
                                                               message_publishers=[],
                                                               miniframe_header=entry.miniframe_header,
                                                               dataframe_header=entry.dataframe_header)

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='check message codec config')
    ap.add_argument("file_name", help="Path to yaml file")
    ap.add_argument("codecs_dir", help="Codecs include directory")

    args = ap.parse_args()

    check_codec_config_yaml(args.file_name, args.codecs_dir)