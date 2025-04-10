from __future__ import annotations
import functools
from typing import Dict, Optional, Any
import typing
if typing.TYPE_CHECKING:
    from ..message_queue_node import MessageQueueNode

import rospy
from ros_acomms_msgs.msg import ReceivedPacket
from bitstring import ConstBitStream, Bits, BitArray, ReadError, BitStream
from .base_packet_codec import BasePacketCodec

# Helper functions
def setattr_nested(obj, attr, value):
    def rgetattr(obj, attr, *args):
        def _getattr(obj, attr):
            return getattr(obj, attr, *args)
        return functools.reduce(_getattr, [obj] + attr.split('.'))

    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, value)


class PacketCodec(BasePacketCodec):
    def __init__(self, message_codecs: Dict, message_publishers=None,
                 miniframe_header=[0xac], dataframe_header=[], **kwargs):
        super(PacketCodec, self).__init__(message_codecs, message_publishers=message_publishers,
                                             miniframe_header=miniframe_header, dataframe_header=dataframe_header)

        self.header_size_bits = 8
        self.codec_overhead_bits = self.header_size_bits + len(miniframe_header) * 8 + len(dataframe_header) * 8

        rospy.loginfo("ROS packet codec initialized with {} message codecs.".format(len(message_codecs)))

    def do_headers_match(self, received_packet: ReceivedPacket):
        if len(self.miniframe_header) > 0:
            if self.miniframe_header != received_packet.packet.miniframe_bytes[:len(self.miniframe_header)]:
                return False
        if len(self.dataframe_header) > 0:
            if self.dataframe_header != received_packet.packet.dataframe_bytes[:len(self.dataframe_header)]:
                return False
        return True

    def encode_payload(self, num_miniframe_bytes: int, num_dataframe_bytes: int, message_queue: 'MessageQueueNode'):
        rospy.logdebug("ROS Packet Codec: Encoding payload")
        remaining_miniframe_bits = num_miniframe_bytes * 8
        remaining_dataframe_bits = num_dataframe_bytes * 8

        # Add headers
        miniframe_bits = BitArray(bytes=bytes(self.miniframe_header))
        remaining_miniframe_bits = max(remaining_miniframe_bits - miniframe_bits.length, 0)
        dataframe_bits = BitArray(bytes=bytes(self.dataframe_header))
        remaining_dataframe_bits = max(remaining_dataframe_bits - dataframe_bits.length, 0)

        total_remaining_bits = remaining_miniframe_bits + remaining_dataframe_bits

        payload_bits = BitArray()
        messages_in_packet = []
        while total_remaining_bits > self.header_size_bits:
            # the packet codec is responsible for adding message headers, to allow flexiblity for sequence
            # numbers, acks, etc.
            next_message = message_queue.get_next_message(
                        max_size_in_bits=total_remaining_bits - self.header_size_bits,
                        packet_codec=self,
                        exclude_messages=messages_in_packet)

            if not next_message:
                break

            message_header, message_bits = next_message.get_next_payload_smaller_than_bits(
                size_in_bits=total_remaining_bits - self.header_size_bits,
                dest_sequence_nums=message_queue.dest_sequence_num,
                remove_from_queue=True,
                )

            if message_header is None:
                break

            messages_in_packet.append(next_message)

            # prepend the message id (header)
            message_bits.prepend(Bits(uint=message_header, length=8))
            payload_bits.append(message_bits)
            total_remaining_bits -= message_bits.length

        miniframe_bits.append(payload_bits[0:remaining_miniframe_bits])
        dataframe_bits.append(payload_bits[remaining_miniframe_bits:])

        miniframe_bytes = miniframe_bits.tobytes()
        dataframe_bytes = dataframe_bits.tobytes()

        rospy.loginfo(
            f"RosPacketCodec encoded {len(messages_in_packet)} msgs in {len(miniframe_bytes) + len(dataframe_bytes)} bytes: {miniframe_bytes.hex()} | {dataframe_bytes.hex()}")

        return messages_in_packet, miniframe_bytes, dataframe_bytes

    def decode_packet(self, received_packet: ReceivedPacket) -> None:
        if not self.message_publishers:
            # This codec is encode-only
            return

        # First, remove the headers
        miniframe_bytes = received_packet.packet.miniframe_bytes[len(self.miniframe_header):]
        dataframe_bytes = received_packet.packet.dataframe_bytes[len(self.dataframe_header):]

        # We glom all of the bytes together and just loop through to decode everything
        payload_bytes = miniframe_bytes + dataframe_bytes
        payload_bits = ConstBitStream(bytes=payload_bytes)

        self.decode_payload(payload_bits, received_packet=received_packet)

    def decode_payload(self, payload_bits: BitStream, received_packet: Optional[ReceivedPacket] = None,
                       src: Optional[int] = None,
                       set_fields: Dict[str, Any] = {},
                       publish_message: bool = True) -> None:
        if not self.message_publishers:
            # This codec is encode-only
            return

        # the format of a ros acomms packet is:
        # ros_acomms header (already stripped)
        # repeat:
        #   message header (id), uint8 (eventually extend this as a varint)
        #   message bits

        # Get the SRC for this packet, which can be set explictly or in the received_packet
        if src is None:
            #rospy.logwarn("missing SRC")
            if received_packet:
                src = received_packet.packet.src

        # loop through until we are out of bits (ReadError will be raised)
        try:
            while True:
                id = payload_bits.read('uint:8')
                rospy.logdebug("Parsed ID: {0}".format(id))
                if self.message_codecs[id % 128] is None:
                    rospy.logdebug("Unknown codec for {0}".format(id % 128))
                    Raise: KeyError

                message_codec = self.message_codecs[id]
                rospy.loginfo("ROS Message decode: ID {} with {}".format(id, message_codec))
                rospy.logdebug(payload_bits)
                ros_msg = self.message_codecs[id].decode(payload_bits, received_packet)
                for field, value in set_fields.items():
                    # Only set the field if the message already has it.  The exception handling covers this.
                    # ROS messages aren't dictionaries, so we can't just add fields.
                    try:
                        setattr_nested(ros_msg, field, value)
                    except (NameError, AttributeError) as e:
                        rospy.loginfo(f"Message decode: could not set {field} on {ros_msg}.  Error: {e}")
                try:
                    if id not in self.message_publishers:
                        rospy.loginfo(f"Decoded message with queue_id: {id}, does not have a publish_topic (this is optional). Doing nothing with it.. ros_msg: {ros_msg}.")
                    elif isinstance(self.message_publishers[id], rospy.Publisher):
                        self.message_publishers[id].publish(ros_msg)
                    else:
                        if src is not None:
                            try:
                                self.message_publishers[id][src].publish(ros_msg)
                            except KeyError:
                                rospy.loginfo("Got a message from an un-named SRC {}, not publishing it".format(
                                    src))
                        else:
                            rospy.logwarn("Got a message that needs metadata to publish, but no metadata was supplied")
                    rospy.logdebug("Published msg id: {0}".format(id))
                except Exception as e:
                    rospy.logerr("Error publishing ROS message: {}".format(e))

        except KeyError:
            # We don't have a config entry that matches this ID
            rospy.loginfo("Unknown message ID ({}) received, unable to continue parsing packet".format(id))
            return
        except ReadError:
            # We are done reading
            pass
        except ValueError:
            # There is a bug in version 4.2.x of BitString that raises ValueError instead of ReadError
            pass
