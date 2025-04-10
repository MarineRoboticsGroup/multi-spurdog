#!/usr/bin/env python3

import rospy
from ros_acomms_modeling.srv import SimPacketPerformance, SimPacketPerformanceResponse

import numpy as np
from numpy.random import default_rng

rate1snr = [-7.95718822832424,
            -7.60350310430977,
            -7.23414218974915,
            -6.84530249111293,
            -6.44364806498048,
            -6.01242663335914]

rate1probability = [1,
                    0.936666666666667,
                    0.726666666666667,
                    0.170000000000000,
                    0.0200000000000000,
                    0]

rate3snr = [-2.9215, 
            -2.2806, 
            -1.5853, 
            -0.8275, 
            0.0044]

rate3probability = [1.0000, 
                    0.9100, 
                    0.5700, 
                    0.1150, 
                    0.0250]

rate5snr = [3.09699410930214,
            4.44256115246717,
            6.01504558096942]

rate5probability = [1,
                    0.135000000000000,
                    0]


class SimPacketPerformanceNode(object):
    def __init__(self):

        rospy.init_node('sim_packet_performance_node', log_level=rospy.DEBUG)
        rospy.loginfo("Starting sim_packet_performance_node...")

        # Get node parameters
        self.fixed_frame_error_rate = rospy.get_param('~fixed_frame_error_rate', None)

        self.random_seed = rospy.get_param('random_seed', None)
        self.rng = default_rng(self.random_seed)

        # Check node parameters
        # TODO: check validity of the parameters here and output contents to log

        sim_service = rospy.Service('sim_packet_performance', SimPacketPerformance, self.handle_sim)

        rospy.spin()

    def handle_sim(self, req):
        rospy.logdebug("Simulating Packet Performance:\r\n" + str(req))

        signal_db = req.rx_level_db
        noise_db = req.noise_level_db
        rate = req.packet_rate

        snr = signal_db - noise_db
        frame_success = []

        if rate == 1:
            if self.fixed_frame_error_rate:
                frame_error_rate = self.fixed_frame_error_rate
            else:
                frame_error_rate = np.interp(snr, rate1snr, rate1probability)
            for i in range(1, 3):
                success = self.rng.uniform() > frame_error_rate
                frame_success.append(success)

        elif rate == 3:
            if self.fixed_frame_error_rate:
                frame_error_rate = self.fixed_frame_error_rate
            else:
                frame_error_rate = np.interp(snr, rate3snr, rate3probability)
            for i in range(1, 2):
                success = self.rng.uniform() > frame_error_rate
                frame_success.append(success)

        elif rate == 5:
            if self.fixed_frame_error_rate:
                frame_error_rate = self.fixed_frame_error_rate
            else:
                frame_error_rate = np.interp(snr, rate5snr, rate5probability)
            for i in range(1, 8):
                success = self.rng.uniform() > frame_error_rate
                frame_success.append(success)

        else:
            rospy.logwarn("Rate: " + str(rate) + " not supported.")

        resp = SimPacketPerformanceResponse(packet_success=all(frame_success),
                                            miniframe_success=[True, True],
                                            frame_success=frame_success,
                                            frame_error_probability=frame_error_rate)

        rospy.logdebug(
            "Simulating Packet Performance Responce:\r\n" + str(resp))
        return resp


if __name__ == '__main__':
    try:
        node = SimPacketPerformanceNode()
        rospy.loginfo("sim_packet_performance_node shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("sim_packet_performance_node shutdown (interrupt)")
