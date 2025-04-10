#!/usr/bin/env python3

from typing import Dict, List, Tuple

import rospy
from ros_acomms_msgs.msg import ReceivedPacket

from acomms_codecs import packet_codecs

from version_node import version_node
from codec_config_parser import QueueParams, QueueId, MessageCodecId, get_queue_params, get_message_codecs, \
    read_packet_codec_yaml, load_custom_codecs


class DecoderListEntry:
    def __init__(self, codec_name, miniframe_header: bytes = bytes(), dataframe_header: bytes = bytes(),
                 packet_codec=None, match_src: List[int] = [], match_dest: List[int] = [],
                 except_src: List[int] = [], except_dest: List[int] = [], remove_headers: bool = True,
                 **kwargs) -> None:
        #TODO: move header stuff into separate packetcodec class
        self.codec_name = codec_name
        self.miniframe_header = miniframe_header
        self.dataframe_header = dataframe_header
        self.packet_codec = packet_codec
        self.match_src = match_src
        self.match_dest = match_dest
        self.except_src = except_src
        self.except_dest = except_dest
        self.remove_headers = remove_headers

    def does_packet_match(self, received_packet: ReceivedPacket) -> bool:
        # We work in this order:
        # 1. SRC
        # 2. DEST
        # 3. Packet type (not yet implemented)
        # 4. miniframe header
        # 5. dataframe header
        src = received_packet.packet.src
        dest = received_packet.packet.dest

        if len(self.match_src) > 0:
            if src not in self.match_src:
                print("SRC doesn't match")
                return False
        if len(self.match_dest) > 0:
            if dest not in self.match_dest:
                print("DEST doesn't match")
                return False
        if len(self.except_src) > 0:
            if src in self.except_src:
                print("SRC excluded")
                return False
        if len(self.except_dest) > 0:
            if dest in self.except_dest:
                print("DEST excluded")
                return False
        if len(self.miniframe_header) > 0:
            if bytes(self.miniframe_header) != received_packet.packet.miniframe_bytes[:len(self.miniframe_header)]:
                if received_packet.packet.miniframe_bytes[:len(self.miniframe_header)] not in [b'', b'\x00']:
                    print("Miniframe header mismatch: {} not {}".format(received_packet.packet.miniframe_bytes[:len(self.miniframe_header)], self.miniframe_header))
                # if we got to this point, this is a zero padded packet or a packet without a payload
                return False
        if len(self.dataframe_header) > 0:
            if bytes(self.dataframe_header) != received_packet.packet.dataframe_bytes[:len(self.dataframe_header)]:
                print("Dataframe header mismatch: {} not {}".format(received_packet.packet.dataframe_bytes[:len(self.dataframe_header)], self.dataframe_header))
                return False
        return True

    def strip_headers(self, received_packet: ReceivedPacket) -> Tuple[bytes, bytes]:
        if self.remove_headers:
            miniframe_bytes = received_packet.packet.miniframe_bytes[len(self.miniframe_header):]
            dataframe_bytes = received_packet.packet.dataframe_bytes[len(self.dataframe_header):]
        else:
            miniframe_bytes = received_packet.packet.miniframe_bytes
            dataframe_bytes = received_packet.packet.dataframe_bytes
        return miniframe_bytes, dataframe_bytes

    def __repr__(self):
        return f"DecoderListEntry: {self.__dict__}"

class PacketDispatchNode(object):
    def __init__(self):
        rospy.init_node('packet_dispatch')

        try:
            version = version_node()
            version.getLtCodecs()
            version.getAcomms()
        except:
            rospy.logwarn("Unable to query version information")


        self.setup_codecs()

        packet_rx_topic = rospy.names.canonicalize_name(rospy.get_param('~packet_rx_topic', 'packet_rx'))
        rospy.Subscriber(packet_rx_topic, ReceivedPacket, self.on_packet_rx)

        rospy.loginfo("Packet Dispatch started")
        rospy.spin()

    def setup_codecs(self) -> None:
        load_custom_codecs()
        packet_codec_param = read_packet_codec_yaml()

        self.decoder_list: List[DecoderListEntry] = []
        for packet_codec in packet_codec_param:
            # QUESTION(lindzey): Storing the message codec dict as a member
            #    variable mirrors the logic that was here before,
            #    but seems to imply that there will only ever be a single
            #    packet codec in the config file. Is that true?
            self.message_codecs = get_message_codecs(packet_codec)
            self.queue_params = get_queue_params(packet_codec)
            # Convert SRC IDs to integers, since yaml dict keys have to be strings
            try:
                src_names: dict = {int(k): v for k, v in packet_codec.get('src_names', {}).items()}
            except Exception as e:
                rospy.logerr(f"Error parsing src_names parameter: {e}")

            self.message_publishers: Dict[QueueId, rospy.Publisher] = {}
            for queue_id, params in self.queue_params.items():
                if params.publish_topic:
                    if ("{src}" in params.publish_topic) or ("{name}" in params.publish_topic):
                        # In this case, we will have a dict of message publishers that correspond to different SRC IDs
                        for src, name in src_names.items():
                            pub_name = params.publish_topic.format(src=src, name=name)
                            pub_name = rospy.names.canonicalize_name(pub_name)
                            if queue_id not in self.message_publishers.keys():
                                self.message_publishers[queue_id] = {}
                            self.message_publishers[queue_id][src] = rospy.Publisher(pub_name, params.msg_class,
                                                                                     queue_size=10)
                    else:
                        pub_name = rospy.names.canonicalize_name(params.publish_topic)
                        self.message_publishers[queue_id] = rospy.Publisher(
                            pub_name, params.msg_class, queue_size=10)

            packet_codec.pop('message_codecs')
            try:
                packet_codec.pop('src_names')
            except:
                pass
            entry = DecoderListEntry(**packet_codec)
            self.decoder_list.append(entry)
            rospy.loginfo("Added decoder: {}".format(entry))

            # replace name with instance of packet codec
            # ToDO make this less ugly and stupid
            entry.packet_codec = packet_codecs[entry.packet_codec].PacketCodec(self.message_codecs,
                                                                   message_publishers=self.message_publishers,
                                                                   **packet_codec)

            rospy.loginfo("Receive packet codec initialized with {} message codecs.".format(len(self.message_codecs)))

    def on_packet_rx(self, received_packet: ReceivedPacket) -> None:
        rospy.loginfo("Got incoming packet")
        # Decide what to do with the incoming packet, based on the match criteria
        matching_codecs = [d.packet_codec for d in self.decoder_list if d.does_packet_match(received_packet)]

        if not matching_codecs:
            rospy.loginfo("Dispatcher received packet that matched no packet decoders. (safe to ignore on zero padded packets)")

        for d in matching_codecs:
            try:
                d.decode_packet(received_packet)
            except KeyError:
                rospy.logwarn("Unrecognized packet decoder: {}".format(d))
            except Exception as e:
                rospy.logerr(f'ERROR Decoding Packet: {(e,)}', exc_info=True)


if __name__ == '__main__':
    try:
        node = PacketDispatchNode()
        rospy.loginfo("Message Dispatch shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("Message Dispatch shutdown (interrupt)")
