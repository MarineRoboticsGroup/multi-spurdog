import rospy
from ros_acomms_msgs.msg import ReceivedPacket
from bitstring import ConstBitStream, Bits, BitArray, ReadError
from .base_packet_codec import BasePacketCodec

class PacketCodec(BasePacketCodec):
    def __init__(self, message_codecs, message_publishers=None,
                 miniframe_header=[], dataframe_header=[], **kwargs):
        super(PacketCodec, self).__init__(message_codecs, message_publishers=message_publishers,
                                             miniframe_header=miniframe_header, dataframe_header=dataframe_header)

        self.header_size_bits = 8


    def encode_payload(self, num_miniframe_bytes, num_dataframe_bytes, message_queue):
        rospy.loginfo("CCL Packet Codec: Encoding payload")
        remaining_miniframe_bits = num_miniframe_bytes * 8
        remaining_dataframe_bits = num_dataframe_bytes * 8

        # Add headers.  CCL messages can't have additional headers on the frame, but we can (theoretically) use
        # miniframe headers in conjunction with the "print_rxd" option on TFP messages.
        miniframe_bits = BitArray(bytes=bytes(self.miniframe_header))
        remaining_miniframe_bits -= miniframe_bits.length
        dataframe_bits = BitArray()

        # CCL is easy, especially as we do it here: one message per packet, always a dataframe.
        next_message = message_queue.get_next_message(
            max_size_in_bits=remaining_dataframe_bits,
            packet_codec=self,
            )

        if not next_message:
            return 0, bytes(), bytes()

        message_queue.mark_transmitted(next_message)

        message_header = next_message.queue.message_codec_id
        message_bits = next_message.payload_bits
        messages_in_packet = [next_message]

        # prepend the message id (header)
        message_bits.prepend(Bits(uint=message_header, length=8))

        dataframe_bits.append(message_bits)

        rospy.loginfo("Encode: {} msgs, {} {}".format(len(messages_in_packet), miniframe_bits.tobytes(),
                                                      dataframe_bits.tobytes()))

        return messages_in_packet, miniframe_bits.tobytes(), dataframe_bits.tobytes()

    def decode_packet(self, received_packet: ReceivedPacket):
        if not self.message_publishers:
            # This codec is encode-only
            return

        # Only parse dataframe bytes
        payload_bits = ConstBitStream(bytes=received_packet.packet.dataframe_bytes)

        try:
            while True:
                id = payload_bits.read('uint:8')

                ros_msg = self.message_codecs[id].decode(payload_bits)
                if isinstance(self.message_publishers[id], rospy.Publisher):
                    self.message_publishers[id].publish(ros_msg)
                else:
                    try:
                        self.message_publishers[id][received_packet.packet.src].publish(ros_msg)
                    except KeyError:
                        rospy.loginfo("Got a message from an un-named SRC {}, not publishing it".format(
                            received_packet.packet.src))
        except KeyError:
            # We don't have a config entry that matches this ID
            rospy.loginfo("Unknown message ID ({}) received, unable to continue parsing packet".format(id))
            return

        except ReadError:
            # We are done reading
            pass