#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from numpy.random import default_rng


class SimpleNoiseModelNode(object):
    def __init__(self):
        rospy.init_node('simple_noise_model')
        rospy.loginfo("Starting simple_noise_model")

        # Get node parameters
        self.mean_noise_db = rospy.get_param('mean_noise_db', 60.0)
        self.noise_stddev_db = rospy.get_param('noise_stddev_db', 3.0)
        self.update_every_seconds = rospy.get_param('update_every_seconds', 10)
        self.random_seed = rospy.get_param('random_seed', None)

        self.noise_publisher = rospy.Publisher(
            'ambient_noise', Float32, queue_size=10)

        self.rng = default_rng(self.random_seed)

        while True:
            noise = self.rng.normal(self.mean_noise_db, self.noise_stddev_db)
            self.noise_publisher.publish(Float32(data=noise))
            rospy.sleep(self.update_every_seconds)


if __name__ == '__main__':
    try:
        node = SimpleNoiseModelNode()
        rospy.loginfo("simple_noise_model shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("simple_noise_model shutdown (interrupt)")
