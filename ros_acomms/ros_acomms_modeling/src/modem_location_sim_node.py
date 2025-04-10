#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from ros_acomms_modeling.msg import Location
from numpy.random import default_rng


class ModemLocationSimNode:
    def __init__(self):
        rospy.init_node('modem_location_sim')
        rospy.loginfo("Starting modem_location_sim_node")
        update_period = rospy.get_param('~period', 5)

        self.simple_path = rospy.get_param('simple_path', False)
        self.random_seed = rospy.get_param('random_seed', None)

        self.min_depth = rospy.get_param('min_depth', 3)
        self.max_depth = rospy.get_param('max_depth', 18)
        self.offset = rospy.get_param('simple_path_offset', 10)

        self._start_lat = 41421580
        self._start_lon = 70726027
        self._end_lat = 41553703
        self._end_lon = 70843443

        self.rng = default_rng(self.random_seed)

        self.location_publisher = rospy.Publisher('location', Location, latch=True, queue_size=10)

        published_origin = False
        while not rospy.is_shutdown():
            if self.simple_path:
                if published_origin:
                    start_lat, start_lon = self.next_start_lat_lon()
                else:
                    start_lat, start_lon = self._start_lat, self.start_lon
                    published_origin = True
                self.pub_path_location(start_lat, start_lon)
            else:
                self.pub_location()
            rospy.sleep(update_period)

    def next_depth(self):
        if self.min_depth > self.max_depth:
            return self.rng.uniform(self.max_depth, self.min_depth)
        return self.rng.uniform(self.min_depth, self.max_depth)

    def next_start_lat_lon(self):
        self._start_lat += self.rng.uniform(-5, 5)
        self._start_lon += self.rng.uniform(-5, 5)
        return self._start_lat, self._start_lon

    def pub_path_location(self, start_lat, start_lon):
        lat = self.rng.uniform(start_lat, start_lat + self.offset) * (10 ** -6)
        lon = self.rng.uniform(start_lon, start_lon + self.offset) * -(10 ** -6)
        depth = self.next_depth()
        msg = Location(header=Header(stamp=rospy.get_rostime()), latitude=lat, longitude=lon, depth=depth)
        self.location_publisher.publish(msg)

    def pub_location(self):
        lat = self.rng.uniform(self._start_lat, self._end_lat) * (10 ** -6)
        lon = self.rng.uniform(self._start_lon, self._end_lon) * -(10 ** -6)
        depth = self.next_depth()
        msg = Location(header=Header(stamp=rospy.get_rostime()), latitude=lat, longitude=lon, depth=depth)
        self.location_publisher.publish(msg)


if __name__ == '__main__':
    try:
        node = ModemLocationSimNode()
        rospy.loginfo("ModemLocationSimNode shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("ModemLocationSimNode shutdown (interrupt)")
