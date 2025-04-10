ros_acomms Overview
===================

ros_acomms is a ROS package that provides a generic ROS interface to the
WHOI Micromodem and transparent transport of ROS messages across an
acoustic link. It also provides simulation capabilities.

Status of ros_acomms
--------------------

ros_acomms is stable enough to be used at sea and has seen significant
deployment time.

As always, you should test your configuration thoroughly before
deployment and have suitable backup plans in place.

Note that the documentation is much more of a work-in-progress than the code is.
We're hoping to have it cleaned up soon.  This section is basically a copy-paste
job from the old README file, and is out of date in places.  The module
documentation is generally up-to-date, but may not be complete yet.

ROS Message Transport Overview
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ros_acomms provides a complete system for transporting messages across
the acoustic link.

.. figure:: /images/data_flow_overview.png
   :alt: Data Flow Overview

   Data Flow Overview

To transmit a ROS message:

1. The user publishes ROS message on a topic
2. The *Message Queue Node* subscribes to that topic.
3. When a new ROS message is received, the *Message Queue Node* uses a
   *Message Codec* to convert the data in the message into a compact
   bit-packet representation by using *Field Codecs* to encode each
   message field, including nested ROS messages.
4. The *Media Access Controller* (for now, the *TDMA MAC*) queries the
   *Message Queue Node* to get the highest priority message for transmit
5. The *Message Queue Node* identifies the highest priority message, and
   then uses the appropriate *Packet Codec* to pack as many messages as
   will fit into one modem *Packet* for transport.
6. The *Media Access Controller* queues the *Packet* for transmit on the
   *Acomms Driver Node*, which interfaces to the modem to actually
   transmit the packet

On the receive side:

1. A packet is received by the modem.
2. The *Acomms Driver Node* handles the incoming packet and publishes it
   as a *ReceivedPacket*
3. The *Packet Dispatch Node* evaluates metadata on the packet (the
   modem SRC, header bytes) and determines which *Packet Codec* to use
   to decode the packet.
4. The *Packet Codec* uses calls one or more *Message Codecs* to decode
   the packed bits into a ROS message
5. The *Packet Dispatch Node* publishes the decoded ROS message on a
   user-specified topic.

When using simulation, the *Acomms Driver Node* is replaced by the
*Modem Sim Node*, but the rest of the flow is the same.

Media Access Controllers (MAC)
------------------------------

ros_acomms supports interchangeable MAC nodes.  Currently, there are several
nodes that support Time-Division-Multiple-Access (TDMA), which sends traffic
during defined timeslots.

The status of the TMDA nodes can be monitored by examining their corresponding status
topics.  (This is intended as an engineering and diagnostics tool... be cautious about
using these topics to drive other behavior in the system).

The "flavors" of TDMA MACs are:

:mod:`tdma_node`
   Basic TDMA node.  This is useful for testing simple systems, and is used
   as a base for more capable nodes.  It only supporst sending comms packets,
   not navigation (no pings).
   This publishes status (:ros_message:`ros_acomms_msgs/TdmaStatus`) on `tdma_status`.
:mod:`tdma_advanced_node`
   Advanced TMDA node.  This inherits from :mod:`tdma_node`, and offers a
   superset of its functionality.  It adds dynamic reconfigure parameters,
   software-mute, and transponder/modem ping capability.  This is generally the
   one you want to use.
   This publishes status (:ros_message:`ros_acomms_msgs/TdmaAdvancedStatus`) on `tdma_advanced_status`.
   (This message includes the basic `TdmaStatus` as a field).
:mod:`tdma_scripted_node`
   Scriped TDMA node.  This inherits from from :mod:`tdma_node`, and offers a
   superset of its functionality.  It adds the ability to use a yaml script file
   to cycle through MAC settings.  This is useful for comms testing.
   This publishes status (:ros_message:`ros_acomms_msgs/TdmaScriptedStatus`) on `tdma_scripted_status`.
   (This message includes the `TdmaAdvancedStatus` as a field).




Node Overview
-------------

A functional ros_acomms system requires instantiating several nodes, as
described in the data flow description above.

Details of some key nodes are highlighted below. Additional details can
be found in the documentation for each node class.

Message Queue Node: - The message queue node is responsible for managing
the message queue and dispatching messages to the appropriate codecs.
The message queue node uses a deque data structure to store messages.
Each topic represented in the message codec config has its message
queue. The message queue node also provides a mechanism for prioritizing
messages, both at configuration time and at runtime. Each topic queue
has its own set of specified parameters including its priority, whether
the message should be fragmented, and many others. These parameters are
documented further in the message codec config portion of this document.
Each of the subscribers specified in the message codec config is created
on startup by the message queue node. The message queue node uses the
codecs to encode incoming messages.

::

   ```
   Subscribers:
     - neighbor_reachability: NeighborReachability
     - from_acomms/encoded_ack: EncodedAck
     - message_codec_params['subscribe_topic'] (A subscriber is dynamically generated for each topic listed in the message codec config)

   Publishers:
     - queue_status

   Services:
     - priority_update: PriorityUpdate (Change the priority of a topic queue)
     - queue_active: QueueActive (Enable or disable a topic queue)
     - get_next_packet_data: GetNextPacketData (Retrieve the highest priority data for transmission)

   Init Parameters:
     - packet_codecs: dictionary (see codecs section)
     - unknown_dests_are_reachable: bool (if true, the message queue assumes all of the destination addresses are reachable. If false, only destinations with known neighbor reachability will be marked as reachable)
     - update_rate: int (rate in hertz at which the queue status is published)
     - default_dest: int (Default 121)
     - default_priority: int (Default 10)
   ```

tdma_node:

-  The TDMA MAC node (time division multiple access / media access
   control) coordinates acoustic transmissions on a user specified
   schedule to allow multiple modems to share the acoustic channel
   without collisions. The TDMA node queries the Message Queue Node to
   get the highest priority message for transmit using the
   get_next_packet_bytes service.

   ::

      Publishers:
        - tdma_status: TdmaStatus

      Init Parameters:
        - num_slots: int (number of time slots used in the network)
        - slot_duration_seconds: int (length of each time slot)
        - cycle_start_time: rostime (reference epoch used to determine time slot start)
        - active_slots: int (length of acoustic packet)
        - guard_time_seconds: int (time between packet transmission)
        - packet_length_seconds: int (length of acoustic packet)
        - always_send_test_data: bool (set to true if TDMA node should send data even when no new message is in the queue)
        - maximum_miniframe_bytes: int (maximum number of miniframe bytes to populate in the transmitted packet)
        - maximum_dataframe_bytes: int (maximum number of dataframe bytes to populate in the transmitted packet)
        - miniframe_rate: int (modem data rate for miniframes)
        - dataframe_rate: int (modem data rate for dataframes)

acomms_driver_node:

-  The TDMA node queries the *Message Queue Node* to get the highest
   priority message for transmit

   ::

      ```
      Subscribers:
        - nmea_to_modem: String (primarily used for debugging, allows user to send NMEA sentences directly to the modem)

      Publishers:
        - cst: CST (modem cycle statistics)
        - packet_rx: ReceivedPacket (packet received from the modem)
        - nmea_from_modem: String (raw NMEA sentences from the modem, primarily used for debugging)

      Init Parameters:
        - modem_connection_type: string (serial (default) / UDP)
        - modem_remote_host: string (remote address of UDP modem)
        - modem_remote_port: int (remote port of UDP modem)
        - modem_local_host: string (local address for UDP connections (default 0.0.0.0))
        - modem_local_port: int (local port for UDP connections (defaults to remote port))
        - modem_serial_port: string (modem serial port for serial connections)
        - modem_baud_rate: int (baud rate for serial connections)
        - set_modem_time: bool (set modem time from host computer on startup)
        - modem_config: dict (dictionary of modem config parameters to be set at startup)
        - publish_partial_packets: bool (incomplete feature: do not use)
        - default_modem_dest: int (modem destinations address to use when no address is set in queue_tx_packet service)
        - use_legacy_packets: bool (force the use of legacy PSK)
        - use_tdp: bool (force the use of TDP packets rather than TFP packets)
      ```

   modem_sensor_data_node:

-  The modem_sensor_data_node periodically queries the modem for sensor data and publishes it on modem_sensor_data (see the ModemSensorData message)

   ::

      Subscribers:
        - nmea_from_modem: String

      Publishers:
        - modem_sensor_data: ModemSensorData
        - nmea_to_modem: String

modem_sim_node:

-  The modem_sim_node simulates the modem.

   ::

      Subscribers:
        - /acoustic_channel: NeighborReachability
        - tick_topic_name: EncodedAck

      Publishers:
        - packet_rx
        - transmit
        - /acoustic_channel: QueueStatus
        - tock_topic_name
        - /from_acomms/srcPublisher

      Parameters:
        - bandwidth_hz: float32 (bandwidth used by simulated modem, default 5000)
        - center_frequency_hz: float32 (center frequency used by simulated modem, default 10000)
        - SRC: int (SRC address used by simulated modem default, 0)
        - /use_tick_time: bool (use tick/tock simulation time-stepping, default False)
        - ambient_noise_db: int (simulated ambient noise, default 60dB)
        - sim_tick_name: string (name of topic to subscribe to for simulation tick messages, default tick)
        - sim_tock_name: string (name of the topic to publisher simulation tock messages on, default tock)
        - modem_location_source: int (default service to get simulated modems location from the read_location service “static” to use static positions specified by latitude longitude and depth parameters)
        - latitude: float32 (static latitude used for simulation)
        - longitude: float32 (static longitude used for simulation)
        - depth: float32 (static depth used for simulation (in meters))

   packet_dispatch_node:

-  The packet_dispatch_node decodes incoming packets and publishes the
   decoded ROS messages on the appropriate topics.

   ::

      Subscribers:
        - packet_rx: ReceivedPacket

      Publishers:
        - pub_name: list (created as specified in the message codec config)

      Parameters:
        - packet_codecs: dictionary (created as specified in the message codec config)

   platform_location_node:

-  The platform_location_node formats incoming data and returns a
   response tuple.

   ::

      Services:
        - read_location: GetPlatformLocation

   sim_packet_performance:

-  The sim_packet_performance simulates packet performance.

   ::

      Services:
        - /sim_packet_performance: SimPacketPerformance

sim_transmission_loss_node:

-  The sim_transmission_loss_node simulates lost transmissions.

   ::

      Services:
        - sim_transmission_loss: SimTransmissionLoss (Gets the receive level and latency for an acoustic transmission, given source level and platform locations)
        - get_optimal_tx_depth: GetOptimalTxDepth (Gets the best depth at which to transmit to maximize receive level at a remote platform)

      Subscribers:
        - sound_speed_profile: SoundSpeedProfile


   version_node:

-  The version_node prints out version strings about ros_acomms and
   accompanying packages such as ltcodecs. helpful_tools, and pyacomms.

test\_ nodes:

-  nodes begining with test\_ are nodes that are used for testing
   ros_acomms. These nodes are not intended to be used in production.

Messages
--------

below is a list of all the ROS messages used by ros_acomms along with
each node they are used within in ros_acomms.

::

   CST (acomms_driver_node, packet_dispatch_node):
   The CST message fields mirror the uModem2 CycleStats (CST) message fields. See the uModem2 User’s Guide for a description of these fields.
       - version_number: int8
       - mode: int8
       - toa: time
       - toa_mode: int8
       - mfd_peak: uint16
       - mfd_pow: int16
       - mfd_ratio: int16
       - mfd_spl: int16
       - agn: int16
       - shift_ain: int16
       - shift_ainp: int16
       - shift_mfd: int16
       - shift_p2b: int16
       - rate_num: int8
       - src: int16
       - dest: int16
       - psk_error: int16
       - packet_type: int8
       - num_frames: int16
       - bad_frames_num: int16
       - snr_rss: int16
       - snr_in: float32
       - snr_out: float32
       - snr_sym: float32
       - mse: float32
       - dqf: int16
       - dop: float32
       - noise: int16
       - carrier: int32
       - bandwidth: int32
       - sequence_number: int32
       - data_rate: int16
       - num_data_frames: int16
       - num_bad_data_frames: int16

   EncodedAck:
   EncodedAck is internally used by the FragmentationTracker to track message fragments.
       - header: Header
       - src: uint8
       - dest: uint8
       - encodec_ack: uint8[]

   FragmentationStatus (fragmentation_tracker_node):
   FragmentationStatus is used primarily for diagnostics related to the message fragmentation subsystem.
       - header: Header
       - sequence_num: uint8
       - fragment_src: uint8
       - fragment_dest: uint8
       - unix_time: uint32
       - payload_size_blocks: uint16
       - block_size_bits: uint8
       - transferred_start: uint16
       - transferred_end: uint16
       - transferred_sections: uint8
       - acked_start: uint16
       - acked_end: uint16
       - acked_sections: uint8

   ModemSensorData (modem_sensor_data_node):
   The ModemSensorData message is used to publish voltages and temperatures measured by the modem hardware.
       - header: Header
       - pwramp_temp: float32
       - pwramp_vbat: float32
       - pwramp_vtransmit: float32
       - pwramp_modem_bat: float32
       - modem_temp: float32

   NeighborReachability (message_queue_node):
   NeighborReachability messages provide a simple means of publishing the availability of other acoustic modems.  It is likely that this message will be replaced in a future version.
       - dest: int16
       - reachable: bool
       - fastest_rate: int16

   NumBtyes:
   This message is deprecated.
     - num_bytes: uint16

   Packet (acomms_driver_node):
   The Packet message represents an acoustic packet.
       - src: int16
       - dest: int16
       - packet_type: int8 (One of: PACKET_TYPE_UNKNOWN PACKET_TYPE_AUTO, PACKET_TYPE_FSK,PACKET_TYPE_LEGACY_PSK, PACKET_TYPE_FDP)
       - miniframe_rate: int8
       - dataframe_rate: int8
       - miniframe_bytes: uint8[]
       - dataframe_bytes: uint8[]

   QueueStatus (message_queue_node):
   The QueueStatus message provides diagnostic insight into the message queue system.
       - header: Header
       - message_count: int32
       - highest_priority: int16
       - queued_dest_addresses: int16[]
       - summary_message_counts: SummaryMessageCount[]

   ReceivedPacket (modem_sim_node, packet_dispatch_node):
   The ReceivedPacket message represents a received acoustic packet.  It includes a Packet message with the packet data as well as metadata and packet statistics.
       - header: Header
       - Packet: packet
       - minibytes_valid: int16[] (This can be used by packet codecs to work with partial packets.  Not recommended for end-users)
       - databytes_valid: int16[] (This can be used by packet codecs to work with partial packets.  Not recommended for end-users)
       - cst: CST

   SimPacket (modem_sim_node, sim_packet_performance):
   SimPacket is used for simulated acoustic packets travelling on the simulated acoustic channel
       - src_latitude: float32
       - src_longitude: float32
       - src_depth: float32
       - src_tx_level_db: float32
       - center_frequency_hz: float32
       - bandwidth_hz: float32
       - transmit_time: time
       - transmit_duration: duration
       - packet: Packet

   SoundsSpeedProfile:
   The SoundSpeedProfile message describes a sound speed profile.
       - header: Header
       - depths: float32[]
       - sound_speed: float32[]

   Source(acomms_driver_node, modem_sim_node):
   The Source message is used to publish a modem SRC ID.
       - source: int8

   SummaryMessageCount (message_queue_node):
   This message is used internally by the QueueStatus message
       - priority: int16
       - dest_address: int16
       - message_count: int32

   TdmaStatus (tdma_Node, xducer_safety_power):
   The TdmaStatus message provides diagnostic information about the TDMA MAC layer.
       - header: Header
       - current_slot: int8
       - we_are_active: bool
       - remaining_slot_seconds: float32
       - remaining_active_seconds: float32
       - time_to_next_active: float32

   Tick (modem_sim_node):
   Tick is used for synchronization with a larger simulation system.
       - header: Header
       - step_id: int32
       - status: uint16
       - duration: duration
       - STATUS_RESET: uint16
       - STATUS_SEQUENCING: uint16
       - STATUS_OVERFLOW: uint16

   Tock (modem_sim_node):
   Tock is used for synchronization with a larger simulation system.
       - header: Header
       - step_id: int32
       - status: uint16
       - node_name: string
       - STATUS_RESET: uint16
       - STATUS_SEQUENCING: uint16
       - STATUS_OVERFLOW: uint16

   TransmittedPacket (modem_sim_node):
   TransmittedPacket represents a transmitted acoustic packet, including a Packet message and the XST data from the Micromodem.
       - header: Header
       - packet: Packet
       - xst: XST

   XST (modem_sim_node):
   The XST message corresponds to the Transmit Statistics (XST) message from the uModem2.  See the uModem2 User’s Guide for details.
       - version_number: int8
       - tot_ymd: time
       - tot_hms: time
       - toa_mode: int8
       - mode: int8
       - probe_len: int16
       - bw_hz: uint32
       - carrier_hz: uint32
       - rate: int16
       - src: int16
       - dest: int16
       - ack: int16
       - nframes_expected: int16
       - nframes_sent: int16
       - pkt_type: int16
       - nbytes: int32

Services
--------

below is a list of all of the services used in ros_acomms along with
each node they are used within in ros_acomms.

::

     GetNextPacketData (tdma_node, xducer_safety_power_control_node, message_queue_node):
     GetNextPacketData is used by MAC nodes to retrieve data from the message_queue_node for transmit.  It retrieves the highest priority messages that fit in the number of bytes specified in num_miniframe_bytes and num_dataframe_bytes.
         Request:
           - num_miniframe_bytes: int32
           - num_dataframe_bytes: int32
           - match_dest: bool
           - dest: int16
           - min_priority: int32
         Response:
           - dest: int16
           - miniframe_bytes: uint8[]
           - dataframe_bytes: uint8[]
           - queued_message_ids: int32[]
           - num_miniframe_bytes: int32
           - num_dataframe_bytes: int32
           - num_messages: int32

     GetOptimalTxDepth (sim_transmission_loss_node):
     GetOptimalTxDepth is used to identify the optimal depth at which to transmit to maximize receive level at a given receiver.
         Request:
         - src_latitude: float32
         - src_longitude: float32
         - src_min_depth: float32
         - src_max_depth: float32
         - src_tx_level_db: float32
         - center_frequency_hz: float32
         - bandwidth_hz: float32
         - rcv_latitude: float32
         - rcv_longitude: float32
         - rcv_depth: float32
         - horizontal_range: float32
       Response:
         - optimal_tx_depth: float32
         - rcv_rx_level_db: float32

     GetPlatformLocation (platform_location_node, modem_sim_node):
     GetPlatformLocation is used to retrieve platform locations in simulation
       Request:
         - name: string
       Response:
         - valid: bool
         - latitude: float32
         - longitude: float32
         - depth: float32

     PingModem (acomms_driver_node):
     PingModem is used to send an acoustic modem ping to a remote modem.  If a reply is heard, the one-way travel time will be reported.
       Request:
         - dest: uint8
         - reply_timeout: float32
       Response:
         - timed_out: bool
         - one_way_travel_time: float32
         - cst: CST

     PingTransponder:
     This service is under development.
       Request:
         - dest_address: uint8
       Response:
         - travel_times: float32[]

     PriorityUpdate (message_queue_node, test_queue_priority_node):
     PriorityUpdate is used to change the priority (typically specified in the message codec config) for a message queue associated with a given topic.
       Request:
         - topic_name: string
         - priority_desired: int32
       Response:
         - response: bool

     QueueActive (message_queue_node, test_queue_priority_node):
     QueueActive is used to enable and disable message queues associated with specified topics at runtime.
       Request:
         - topic_name: string
         - set_active: bool
       Response:
         - success: bool

     QueueTxPacket:
     QueueTxPacket is used to transmit packets with the acoustic modem or acoustic modem simulator.
       Request:
         - QUEUE_IMMEDIATE: int8
         - QUEUE_PPS: int8
         - QUEUE_TXTRIG: int8
         - queue: int8
         - insert_at_head: bool
         - requested_src_level_db: float32
         - packet: Packet
       Response:
         - success: bool
         - posistion_in_queue: int32
         - actual_src_level_db: float32

     SimPacketPerformance (modem_sim_node, sim_packet_performance_node):
     SimPacketPerformance is used to simulate the success or failure of a simulated acoustic packet given the receive level, noise level, and packet rate.  The probability of success is based on empirical data.
       Request:
         - rx_level_db: float32
         - noise_level_db: float32
         - packet_rate: float32
       Response:
         - packet_success: bool
         - miniframe_success: bool[]
         - frame_succesS: bool[]

     SimTransmissionLoss (sim_transmission_loss_node, test_sim_transmission_loss_node, modem_sim_node):
     SimTransmissionLoss is used to get the receive level and latency of a simulated acoustic path.
       Request:
         - src_latitude: float32
         - src_longitude: float32
         - src_depth: float32
         - src_tx_level_db: float32
         - center_frequency_hz: float32
         - bandwidth_hz: float32
         - rcv_latitude: float32
         - rcv_depth: float32
       Response:
         - rcv_rx_level_db: float32
         - transmission_delay: duration

Codecs
------

The acoustic link is both high-latency and low-throughput. These
limitations are mostly due to physics, and therefore unavoidable. As a
result, it is important to efficiently pack any data you want to send
via acomms.

ros_acomms provides a flexible mechanism for doing this, built of three
layers of *codecs* (an amalgam of “encoder” and “decoder”):

1. Field codecs
2. Message codecs
3. Packet codecs

Field codecs is available in the ltcodec repository, and can be
installed via pip on PyPi

Each field in a message represent something: a numeric value, a string,
an array of some other type, or a nested message. The goal of each field
codec is to represent those values in as few bits as possible. In the
case of ROS messages, each ROS message primititive type has a default
field codec that does a decent job of packing the value into bits.

However, you can use fewer bits to encode the value if you provide more
information about the data that goes in the field. Different types
(floats, integers, strings) have different parameters that you can
specify on the field. For example, the smallest integer field used by
ROS is an int8, which takes 8 bits. However, if you specified that a
field has a minimum value of 0 and a maximum value of 6, it can be
represented using only 3 bits. The field codecs can do this. See the
documenation on each field codec for a description of the parameters
that can reduce the size of encoded values.

Some field codecs are recursive, so that nested message types are
supported.

The message codec is responsible for packing the output of the field
codecs into a single sequence of bits that represents a single message
and vice-versa.

The packet codec takes the output of multiple message codecs and builds
an acomms packet that contains one or more messages, and does the
inverse on the receive side.

Field codecs, message codecs, and packet codecs are all classes that
derive from a base FieldCodec, MessageCodec, or PacketCodec class.
Writing new codecs is intentionally simple.

There is an important distinction between the message codec system used
here and that used by other systems, such as Protobuf, Msgpack, or CBOR.
This codec system optimizes for size over descriptiveness. From a given
packed sequence of bits, there is no way to decode it without knowing
the encoding scheme.

ltcodecs includes all of the message types supported by ros messages.
Each codec type requires its own set of configurable parameters that
need to be specified in the message codec config. In order to interface
with ros_acomms you will need to create codec config file. In your
message codec config file the below parameters will need to be included:

::

   - codec_name: default
     match_src: []
     match_dest: []
     except_src: []
     except_dest: []
     packet_codec: ros
     miniframe_header: []
     dataframe_header: []
     remove_headers: bool
     message_codecs: []

Under the message codecs section of your message codec config file an
entry will need to be made for every publish / subscriber paired that
will be sent through ros_acomms. Each entry in the list will need the
following fields:

::

     - id: int
       message_codec: default
       subscribe_topic: string
       publish_topic: string
       ros_type: string
       default_dest: int
       queue_order: fifo/lifo
       queue_maxsize: int
       is_active: bool
       allow_fragmentation: bool
       priority: int
       fields:
         $field_name:
           codec: $CodecName
           $parameters

An example message codec config can be seen below:

.. container::

   .. figure:: /images/queueConfig.png
      :alt: Config File

      Config File

All of the codec types and associated parameters(\* Indicates Required
Fields) can be seen below:

::

   bool:
     - none

   bytes:
   This codec is included for compatibility with systems using CCL messages. It is not recommended for use with new systems.

   - max_length: int\* (Maximum number of bytes that can be encoded)

   ccl_latlon_bcd:
   This codec is included for compatibility with systems using CCL messages. It is not recommended for use with new systems.

   - lat_not_lon: bool (default: true)

   ccl_latlon:
   This codec is included for compatibility with systems using CCL messages. It is not recommended for use with new systems.

   linspace_float, linspace:
   The linspace codec encodes floats as the nearest value among a specified number of linearly-spaced values. Only one of resolution, num_values, or num_bits may be specified.

   - min_value: float
   - max_value: float
   - resolution: float ((default: none) Spacing between encoded values)
   - num_values: int ((default: none) Number of encoded values between min and max values)
   - num_bits: int ((default: none) Number of bits to use to encode values between min and max values)

   integer:

   - min_value: int\*
   - max_value: int\*
   - resolution: int ((default 1) Can be set to values other than 1 to reduce resolution and decrease encoded size.)
   - little_endian: bool (default: false)

   int8, int16, int32, int64:
   Fixed-length integer codecs, no user-specified parameters.

   uint8, uint16, uint32, uint64:
   Fixed-length unsigned integer codecs; no user-specified parameters

   float:

   - min_value: float\*
   - max_value: float\*
   - precision: int\* (Default 0, number of significant digits after decimal point.)

   float32, float64:
   IEEE floating point formats; no user-specified parameters

   string, ascii:

   - max_length: int (default: 128)
   - bits_per_char: int ((default: 7) Defaults to 7-bit ASCII. 6 will use AIS-standard 6-bit ASCII to encode strings (no lowercase letters, limited symbols)
   - tail: bool ((default: false) If the string exceeds max_length, it will be truncated. If tail is true, the end of the string will be sent rather than the beginning.)

   msg, ros_msg:
   This encodes a ROS message, and must contain a fields dict with types

   - ros_type: str\*
   - fields: dict (default: none)

   variable_len_array:

   - element_type: str\*
   - max_length: int\*
   - element_paramets: str

   fixed_len_array:

   - element_type: str\*
   - length: int\*
   - element_paramets: str (default: none)

   padding, pad:

   - num_bits: int\*

   time, rostime:

   - precision: int (default: 0)
   - epoch_start: int (default: 1622520000)
   - epoch_end: int (default: (2\*\*31 - 1))

Modem Simulator Quick Start
---------------------------

You can use the included Dockerfile to build a Docker container,
download a pre-built Docker image, or just check out this repository and
run one of the example launch files.

Install: 1. Create a ROS workspace, if you don’t already have one. See
the ROS documentation for more information. 2. Clone ros_acomms:
``git clone https://git.whoi.edu/acomms/ros_acomms.git`` 3. Change
directory into ros_acomms ``cd ros_acomms`` 4. Run setup.bash
``pip install -r requirements.txt`` 5. Rebuild your workspace with
``catkin_make`` 6. Run the modem_sim_test launch file from a console. It
will print messages to stdout. Assuming you are in your ROS workspace
root: ``roslaunch src/ros_acomms/src/launch/modem_sim_test.launch``

Modify this launch file to change parameters, spawn additional modems,
etc. To display version information make sure to include the
version_node in your launch file.

To interface with ros_acomms first copy queue_test.launch and
message_codec_config.yaml from /launch and put them into your own ROS
repo. The message_codec_config.yaml file will need to be adjusted to fit
your messages. Make sure to edit the subscribe_topic, publish_topic, and
ros_type fields to match your namespaces. Additionally the fields key
will need to be updated for each parameter within your message. Each
message should have its own message_codec within the config. All
unnecessary message configs can be safely removed.

Info codec typing can be found at
/src/acomms_codes/ltcodecs/**init**.py. Further exploration of
parameters / defaults can be found in the specific codec type files held
within the ltcodecs directory.

If your message is being sent / recived more than once it is likely the
way you have your namespaces set up. One solution is to remove
message_queue_node from one of the modems and packet_dispatch from the
other. Additionally, you can use namespaces like /modem0/command.

Differences between the modem driver and modem sim
--------------------------------------------------

Beyond the obvious differences (the sim supports simulation, including
sim time and synchronization), there are a few differences: 1. The sim
does not currently support the tx_inhibit dynamic reconfigure parameter.

Message Fragmentation and File Transfer
---------------------------------------

ros_acomms implements a system for fragmenting larger messages to send
them across the acoustic link. If the fragmentation parameter is enabled
in the message codec config (discussed in part 5), larger will be
fragmented and sent in pieces. The fragmentation tracker node is
responsible for making sure all the individual pieces of the fragmented
message are correctly transmitted. Once received, the pieces of the
message are reassembled and published on the topic specified in the
message codec config.

By default, the fragmentation system is configured to allow transmitting
messages up to about 65500 bytes in size. It can be configured to send
larger messages, but this is likely not advisable over acoustic links.
Even 64kB messages may prove difficult to transmit within a reasonable
time, as a typical modem packet ranges in size from a few hundred bytes
to about 2kB.

Files can be transferred across an acoustic link using ros_acomms by
packing them in a uint8[] field of a ROS message. Python, C, JPG, PNG,
or any other types of files can be parsed using Python and then packed
into a message that can be sent through ros_acomms. Files (messages)
that are too large to fit in a single modem packet will be fragmented as
long as message fragmentation is enabled in the message codec config.
\``\`
