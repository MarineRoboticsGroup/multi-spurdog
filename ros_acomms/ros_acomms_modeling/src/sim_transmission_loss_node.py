#!/usr/bin/env python3

import rospy
from ros_acomms_modeling.srv import SimTransmissionLoss, SimTransmissionLossRequest, SimTransmissionLossResponse, \
    GetOptimalTxDepth, GetOptimalTxDepthResponse, SimTravelTime, SimTravelTimeRequest, SimTravelTimeResponse
from ros_acomms_msgs.msg import SoundSpeedProfile

import numpy as np
from sys import float_info as fi
import whoi_uwapm.uwapm as pm
from geopy.distance import distance
import ssp_utils

models = ['bellhop', 'bellhopcxx']

class SimTransmissionLossNode(object):
    def __init__(self):

        rospy.init_node('sim_transmission_loss_node', log_level=rospy.INFO)
        rospy.loginfo("Starting sim_transmission_loss_node...")

        # Get node parameters
        self.model = rospy.get_param('~model', default='bellhopcxx')
        self.sound_speed = rospy.get_param('~sound_speed', default=1500)
        self.water_depth = rospy.get_param('~water_depth', default=1000)
        self.bottom_type = rospy.get_param('~bottom_type', default='sand')
        self.plot = rospy.get_param('~plot', default=False)
        self.bellhop_env_nbeams = rospy.get_param(
            '~bellhop_env_nbeams', default=3000)
        self.bellhop_transmission_loss_mode = rospy.get_param(
            '~bellhop_transmission_loss_mode', default='incoherent')
        self.use_bellhop_for_latency = rospy.get_param(
            '~use_bellhop_for_latency', default=False)

        # use_water_depth_from_ssp when True, will use a valid water_depth in ssp message or,
        # .. last depth value in ssp_message.depths for water depth. use_water_depth_from_ssp=False preserves the old behavior
        self.use_water_depth_from_ssp = rospy.get_param('~use_water_depth_from_ssp', default=False)
        ssp_file_name = rospy.get_param('~ssp_file_name', default=None)
        if ssp_file_name is not None:
            try:
                self.sound_speed = ssp_utils.get_ssp_from_csv_file(ssp_file_name)
            except Exception as e:
                rospy.logerr(f"Error processing SSP CSV file ({ssp_file_name}, using fixed sound speed: {e}")

        if self.model in models:
            # Check node parameters
            # TODO: check validity of the parameters here and output contents to log

            self.env = pm.create_env2d(name='SimTransmissionEnvironment',
                                       # [dB/wavelength]
                                       bottom_absorption=0.1,
                                       bottom_density=1600,         # [kg/m^3]
                                       bottom_roughness=0,          # [m (RMS)]
                                       bottom_soundspeed=1600,      # [m/s]
                                       depth=1000,                  # [m]
                                       # [curvilinear/linear]
                                       depth_interp='linear',
                                       # [Hz]                          ###### Changes
                                       frequency=10000,
                                       max_angle=80,                # [deg]
                                       min_angle=-80,               # [deg]
                                       # [Number of beams 0 = auto]
                                       nbeams=0,
                                       # [m]                           ###### Changes
                                       rx_depth=10,
                                       # [m]                           ###### Changes
                                       rx_range=1000,
                                       soundspeed=1500,             # [m/s]
                                       # [spline/linear]
                                       soundspeed_interp='spline',
                                       # [surface profile]
                                       surface=None,
                                       # [curvilinear/linear]
                                       surface_interp='linear',
                                       # [m]                           ######  Changes
                                       tx_depth=5,
                                       # [(deg, dB)...]
                                       tx_directionality=None,
                                       type='2D'                    # [2D/3D]
                                       )

            if self.sound_speed is not None:
                # TODO: Parse sound speed arrays and pass into environment
                self.env['sound_speed'] = self.sound_speed

            if self.water_depth is not None:
                rospy.logdebug("Parameter water_depth: " +
                               str(self.water_depth))
                self.env['depth'] = self.water_depth

            if self.bellhop_env_nbeams is not None:
                rospy.logdebug("Parameter bellhop env nbeams: " +
                               str(self.bellhop_env_nbeams))
                self.env['nbeams'] = self.bellhop_env_nbeams

            pm.print_env(self.env)

            # Check that the environment is sane
            try:
                pm.check_env2d(self.env)
            except ValueError as e:
                rospy.logerr(
                    "Error in Bellhop environment: {}\nUsing environment:\n{}".format(e, self.env))
        else:
            # unknown model
            rospy.logerr(
                "Unknown or default model specified, modeled RL will match TL")
            pass

        # Subscribe to sound speed profile updates, if they are available
        self.ssp_subscriber = rospy.Subscriber(
            'sound_speed_profile', SoundSpeedProfile, self.on_ssp_update)

        sim_service = rospy.Service(
            'sim_transmission_loss', SimTransmissionLoss, self.handle_sim_transmission_loss)
        rospy.Service('sim_travel_time', SimTravelTime, self.handle_sim_travel_time)
        optimal_tx_depth_service = rospy.Service(
            'get_optimal_tx_depth', GetOptimalTxDepth, self.handle_optimal_tx_depth)

        rospy.loginfo("sim_transmission_loss node running.")

        rospy.spin()

    def on_ssp_update(self, ssp_message):
        # we need to rearrange the vectors for use by bellhop
        # ssp_array = np.hstack(
        #     [np.asarray(ssp_message.depths), np.asarray(ssp_message.sound_speeds)])
        try:
            ssp_array = ssp_utils.handle_surface_gap(depths=ssp_message.depths, speeds=ssp_message.sound_speeds)
        except:
            rospy.logerr(f'ERROR: Problem using ssp_utils.handle_surface_gap(). Falling back to old method')
            ssp_array = np.hstack(
                [np.asarray(ssp_message.depths), np.asarray(ssp_message.sound_speeds)])

        self.env['sound_speed'] = ssp_array
        if self.use_water_depth_from_ssp:
            try:
                # if passed water depth is <= 0, use last value in list of depths as water depth
                if float(ssp_message.water_depth) <= 0:
                    self.env['depth'] = float(ssp_message.depths[-1])
                else:
                    self.env['depth'] = float(ssp_message.water_depth)
            except:
                rospy.logerr(f'ERROR: Problem using water_depth. Falling back on self.water_depth')
                self.env['depth'] = self.water_depth
        self.sound_speed = ssp_array

    def handle_sim_transmission_loss(self, req: SimTransmissionLossRequest):
        rospy.logdebug("Simulating Transmission Loss:\r\n" + str(req))

        # Convert lat,lon to compute range
        src = (req.src_latitude, req.src_longitude)
        rcv = (req.rcv_latitude, req.rcv_longitude)

        # publish a warning if the source and receiver are at the same position
        if src == rcv:
            rospy.logwarn("Source and receiver are in exactly the same place.  This is suspicious.")

        horizontal_range = distance(src, rcv, ellipsoid='WGS-84').meters

        receive_level_db = self.get_receive_level(horizontal_range, req.src_depth, req.rcv_depth,
                                                  req.center_frequency_hz, req.src_tx_level_db)

        # Compute arrival time and convert to latency
        if self.use_bellhop_for_latency:
            latency = self.get_bellhop_latency(
                horizontal_range, req.src_depth, req.rcv_depth, req.center_frequency_hz)
        else:
            latency = self.get_geometric_latency(
                horizontal_range, req.src_depth, req.rcv_depth)

        rospy.logdebug("Range:      " + str(horizontal_range))
        rospy.logdebug("Latency:    " + str(latency))

        # Return receive level dB and transmission delay
        return SimTransmissionLossResponse(rcv_rx_level_db=receive_level_db,
                                           transmission_delay=rospy.Duration.from_sec(latency))

    def handle_sim_travel_time(self, req: SimTravelTimeRequest) -> SimTravelTimeResponse:
        rospy.logdebug(f"Simulating Travel Time: {req}")
        # Convert lat,lon to compute range
        src = (req.src_latitude, req.src_longitude)
        rcv = (req.rcv_latitude, req.rcv_longitude)

        horizontal_range = distance(src, rcv, ellipsoid='WGS-84').meters

        # Compute arrival time and convert to latency
        if self.use_bellhop_for_latency:
            travel_time = self.get_bellhop_latency(
                horizontal_range, req.src_depth, req.rcv_depth, req.center_frequency_hz)
            model = 'bellhop'
        else:
            travel_time = self.get_geometric_latency(
                horizontal_range, req.src_depth, req.rcv_depth)
            model = 'geometric'

        return SimTravelTimeResponse(travel_time=rospy.Duration.from_sec(travel_time), model=model)

    def handle_optimal_tx_depth(self, req):
        rospy.logdebug("Calculating optimal TX depth:\r\n" + str(req))

        # Convert lat,lon to compute range
        if req.horizontal_range <= 0:
            src = (req.src_latitude, req.src_longitude)
            rcv = (req.rcv_latitude, req.rcv_longitude)
            horizontal_range = distance(src, rcv, ellipsoid='WGS-84').meters
        else:
            horizontal_range = req.horizontal_range

        best_depth, best_rl = self.get_optimal_tx_depth(horizontal_range=horizontal_range,
                                                        min_depth=req.src_min_depth,
                                                        max_depth=req.src_max_depth,
                                                        dest_depth=req.rcv_depth,
                                                        center_frequency=req.center_frequency_hz,
                                                        tx_level_db=req.src_tx_level_db,
                                                        num_depths=10)

        # Return optimal depth and corresponding receive level
        return GetOptimalTxDepthResponse(rcv_rx_level_db=best_rl, optimal_tx_depth=best_depth)

    def get_receive_level(self, horizontal_range, src_depth, dest_depth, center_frequency=10000, tx_level_db=185):
        if self.model not in models:
            rospy.logerr(f"Unknown model '{self.model}' can't be used to get receive level; returning 0dB")
            return 0

        if horizontal_range <= 0.1:
            rospy.logwarn(f"Horizontal range is near 0; setting receive level to transmit level ({tx_level_db} dB)")
            return tx_level_db

        if src_depth >= self.water_depth:
            rospy.logerr(f"Source depth ({src_depth}) is greater than water depth ({self.water_depth}).  Can't calculate receive level; returning 0dB")
            return 0

        if dest_depth >= self.water_depth:
            rospy.logerr(f"Receiver depth ({dest_depth}) is greater than water depth ({self.water_depth}).  Can't calculate receive level; returning 0dB")
            return 0

        if src_depth <= 0:
            rospy.logwarn(f"Source depth <= 0; setting it to 0.1 to calculate receive level")
            src_depth = 0.1

        if dest_depth <= 0:
            rospy.logwarn(f"Receiver depth <= 0; setting it to 0.1 to calculate receive level")
            dest_depth = 0.1

        return self.get_receive_level_bellhop(horizontal_range, src_depth, dest_depth,
                                              center_frequency, tx_level_db)


    def get_receive_level_bellhop(self, horizontal_range, src_depth, dest_depth, center_frequency=10000, tx_level_db=185):
        # Modify environment file
        self.env['tx_depth'] = src_depth
        self.env['rx_depth'] = dest_depth
        self.env['rx_range'] = horizontal_range
        self.env['frequency'] = center_frequency

        # Run bellhop transmission loss
        time1 = rospy.get_time()
        tloss = pm.compute_transmission_loss(
            self.env, mode=self.bellhop_transmission_loss_mode, model=self.model)
        time2 = rospy.get_time()
        rospy.logdebug("Bellhop transmission loss takes [" + str(self.bellhop_env_nbeams) + " beams]: " + str(
            time2 - time1) + " seconds")

        # Convert complex transmission loss to dB
        tloss_db = 20 * np.log10(fi.epsilon +
                                 np.abs(np.flipud(np.atleast_1d(tloss))))
        receive_level = tx_level_db + tloss_db
        return receive_level

    def get_bellhop_latency(self, horizontal_range, src_depth, dest_depth, center_frequency=10000):
        # Modify environment file
        self.env['tx_depth'] = src_depth
        self.env['rx_depth'] = dest_depth
        self.env['rx_range'] = horizontal_range
        self.env['frequency'] = center_frequency

        time1 = rospy.get_time()
        arrivals = pm.compute_arrivals(self.env, model=self.model)
        latency = arrivals['time_of_arrival'].iloc[0]
        time2 = rospy.get_time()
        rospy.logdebug("Bellhop arrival times takes: " +
                       str(time2 - time1) + " seconds")

        return latency

    def get_geometric_latency(self, horizontal_range, src_depth, dest_depth):
        dist = np.sqrt(horizontal_range ** 2 +
                       np.abs(src_depth - dest_depth) ** 2)
        if np.isscalar(self.sound_speed):
            latency = dist / self.sound_speed
        else:
            latency = dist / np.mean(self.sound_speed[:, 1])
        return latency

    def get_receive_levels_for_tx_depths(self, horizontal_range, src_depths, dest_depth, center_frequency=10000, tx_level_db=185):
        receive_levels = []
        for tx_depth in src_depths:
            rl = self.get_receive_level(
                horizontal_range, tx_depth, dest_depth, center_frequency, tx_level_db)
            receive_levels.append(rl)
        return np.array(receive_levels)

    def get_optimal_tx_depth(self, horizontal_range, min_depth, max_depth, dest_depth, center_frequency=10000, tx_level_db=185, num_depths=10):
        test_depths = np.linspace(min_depth, max_depth, num_depths)
        receive_levels = self.get_receive_levels_for_tx_depths(
            horizontal_range, test_depths, dest_depth, center_frequency, tx_level_db)
        best_idx = np.argmax(receive_levels)
        best_depth = test_depths[best_idx]
        best_rl = receive_levels[best_idx]
        return best_depth, best_rl


if __name__ == '__main__':
    try:
        node = SimTransmissionLossNode()
        rospy.loginfo("sim_transmission_loss_node shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("sim_transmission_loss_node shutdown (interrupt)")
