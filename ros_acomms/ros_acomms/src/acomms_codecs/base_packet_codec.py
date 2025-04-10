from abc import ABC, abstractmethod
from ros_acomms_msgs.msg import ReceivedPacket


class BasePacketCodec(ABC):
    def __init__(self, message_codecs, message_publishers=None, miniframe_header=[], dataframe_header=[], **kwargs):
        self.message_codecs = message_codecs
        self.message_publishers = message_publishers
        self.miniframe_header = miniframe_header
        self.dataframe_header = dataframe_header
        self.codec_overhead_bits = 0

        for id, message_codec in self.message_codecs.items():
            message_codec.packet_codec = self

    def do_headers_match(self, received_packet: ReceivedPacket):
        if len(self.miniframe_header) > 0:
            if self.miniframe_header != received_packet.packet.miniframe_bytes[:len(self.miniframe_header)]:
                return False
        if len(self.dataframe_header) > 0:
            if self.dataframe_header != received_packet.packet.dataframe_bytes[:len(self.dataframe_header)]:
                return False
        return True

    @abstractmethod
    def encode_payload(self, num_miniframe_bytes: int, num_dataframe_bytes: int, message_queue) -> (int, bytes, list):
        pass

    @abstractmethod
    def decode_packet(self, received_packet: ReceivedPacket):
        pass