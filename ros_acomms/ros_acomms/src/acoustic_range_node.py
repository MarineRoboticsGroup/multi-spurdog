#!/usr/bin/env python3

import rospy
from ros_acomms_msgs.msg import PingReply, PingTranspondersReply, CST, AcousticRange
import roslib.message
from rospy import AnyMsg
from std_msgs.msg import Header, Float32

from mac_utils import deserialze_anymsg

class AcousticRangeNode(object):
    """
    """
    def __init__(self):
        rospy.init_node('acoustic_range_node') #, log_level=rospy.DEBUG)
        # starting sound_speed in m/s. When AnyMsg is published on the ~sound_speed_topic, 
        # .. we update this value. If the msg on ~sound_speed_topic does not have sound_speed, 
        # .. this doesn't change and we'll print an error
        self.sound_speed = rospy.get_param('~sound_speed', default=1500)
        # while None, AcousticRange.speed_of_sound_source == SPEED_OF_SOUND_SOURCE_STATIC
        # as soon as we get an update with a sound_speed, we get the time 
        self.last_sound_speed_update = None
        self.sound_speed_max_age_sec = rospy.get_param('~sound_speed_max_age_sec', default=500)
        
        # publisher for acoustic ranges
        self.acoustic_range_publisher = rospy.Publisher(
                rospy.get_param('~acoustic_range_topic', default='acoustic_range'), 
                AcousticRange, 
                queue_size=2, 
                latch=True
            )

        #
        # set up subscriber for topic with any msg as long as it has a field "sound_speed"
        # .. it assumed a float value in m/s.
        rospy.Subscriber(
                rospy.get_param('~sound_speed_topic', default='ctd'), 
                AnyMsg, self.on_sound_speed_update)

        # setup manual subscriber which is useful for spoofing a float value on a topic for sound_speed
        rospy.Subscriber(
                rospy.get_param('~manual_sound_speed_topic', default='~manual_sound_speed'), 
                Float32, self.on_manual_sound_speed)

        # set up subscriber for TWTT modem pings
        rospy.Subscriber(
                rospy.get_param('~ping_reply_topic', default='ping_reply'), 
                PingReply, self.on_ping_reply)

        # set up subscriber for TWTT transponder pings
        rospy.Subscriber(
                rospy.get_param('~ping_transponders_reply_topic', default='ping_transponders_reply'), 
                PingTranspondersReply, self.on_ping_transponders_reply)

        rospy.logwarn(f'acoustic_range_node started')
        self.spin()

    @property
    def sound_speed_age(self):
        if self.last_sound_speed_update is None:
            return self.last_sound_speed_update

        return rospy.Time.now().secs - self.last_sound_speed_update.secs

    def on_manual_sound_speed(self, msg):
        self.sound_speed = float(msg.data)
        self.last_sound_speed_update = rospy.Time.now()

    def on_sound_speed_update(self, msg):
        msg = deserialze_anymsg(msg)
        if 'sound_speed' in msg.__slots__:
            self.sound_speed = float(msg.sound_speed)
            self.last_sound_speed_update = rospy.Time.now()

    def on_ping_reply(self, msg):
        if msg.owtt <= 0.0:
            rospy.logwarn(f'Got a ping_reply with a owtt of 0.0... not publishing range: {msg}')
            return

        acoustic_range_msg = self.generate_acoustic_range(
                measurement_type=AcousticRange.MEASUREMENT_TYPE_TWTT, 
                src=msg.src, 
                dest=msg.dest, 
                owtt=msg.owtt,
            )
        acoustic_range_msg.cst = msg.cst

        rospy.loginfo(f'Publishing Acoustic Range from TWTT modem ping: {acoustic_range_msg}')
        self.acoustic_range_publisher.publish(acoustic_range_msg)

    def on_ping_transponders_reply(self, msg):
        for i, dt_id in enumerate(['owtt_a', 'owtt_b', 'owtt_c', 'owtt_d']):
            # for each DT A-D we publish range if owtt is not 0.0
            owtt = getattr(msg, dt_id, 0.0)

            if owtt:
                acoustic_range_msg = self.generate_acoustic_range(
                        measurement_type=AcousticRange.MEASUREMENT_TYPE_TWTT, 
                        src=-(i + 1), # -1==A, -2==B, -3==C, -4==D
                        dest=-1, 
                        owtt=float(owtt),
                    )

                rospy.loginfo(f'Publishing Acoustic Range from TWTT transponder ping: {acoustic_range_msg}')
                self.acoustic_range_publisher.publish(acoustic_range_msg)

    def generate_acoustic_range(self, measurement_type, src, dest, owtt):
        speed_of_sound_source = AcousticRange.SPEED_OF_SOUND_SOURCE_UNKNOWN
        if self.sound_speed_age is None:
            speed_of_sound_source = AcousticRange.SPEED_OF_SOUND_SOURCE_STATIC
        elif self.sound_speed_age < self.sound_speed_max_age_sec:
            speed_of_sound_source = AcousticRange.SPEED_OF_SOUND_SOURCE_TOPIC
        else:
            speed_of_sound_source = AcousticRange.SPEED_OF_SOUND_SOURCE_INVALID

        acoustic_range_msg = AcousticRange(header=Header(stamp=rospy.Time.now()),
                                           measurement_type=measurement_type,
                                           speed_of_sound_source=speed_of_sound_source,
                                           src=src,
                                           dest=dest,
                                           owtt=owtt,
                                           range_m=float(self.sound_speed * owtt),
                                           speed_of_sound_ms=self.sound_speed)
        return acoustic_range_msg

    def spin(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    try:
        acoustic_range_node = AcousticRangeNode()
        rospy.loginfo("acoustic_range_node shutdown (interrupt)")
    except rospy.ROSInterruptException:
        rospy.loginfo("acoustic_range_node shutdown (interrupt)")


