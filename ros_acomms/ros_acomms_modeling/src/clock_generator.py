#! /usr/bin/env python3

"""
clock_generator.py

This script contains the clock_generator class which publishes a clock message at a specified rate
"""

import rospy
from rosgraph_msgs.msg import Clock
import time
import datetime

class ClockGenerator:
    """
    clock_generator publishes a clock message at a specified rate

    This can drastically speed up the simulation time, which is useful for unit testing
    """
    def __init__(self) -> None:
        rospy.init_node('clock_generator')

        clock_publisher = rospy.Publisher('clock', Clock, queue_size=10)
        rate = rospy.get_param("~publish_rate", 50)
        sim_speed_multiplier = rospy.get_param("~multiplier", 5)
        start_sim_at_current_time = rospy.get_param("~start_sim_at_current_time", False)
        sim_clock_msg = Clock()

        if not start_sim_at_current_time:
            # starts the sim from 0.0 (rospy.get_time() returns 0.0 before msg is published on /clock)
            current_time = rospy.get_time()
        else:
            # rather than start the sim from 0, we can start it relative to launch
            current_time = datetime.datetime.now().timestamp()
            sim_clock_msg.clock = rospy.Time.from_sec(current_time)
            clock_publisher.publish(sim_clock_msg)

        while not rospy.is_shutdown():
            current_time += (1 / rate) * sim_speed_multiplier
            sim_clock_msg.clock = rospy.Time.from_sec(current_time)
            clock_publisher.publish(sim_clock_msg)
            time.sleep(1 / rate)

if __name__ == '__main__':
    try:
        ClockGenerator()
        rospy.loginfo("clock_generator shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("clock_generator shutdown (interrupt)")
