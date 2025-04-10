            self.xst_logs = []
        self.cst_logs = []
        # Logging Subscribers
        self.xst = rospy.Subscriber("modem/xst", XST, self.on_xst)
        self.cst = rospy.Subscriber("modem/cst", CST, self.on_cst)
# Logging:
    def on_xst(self, msg: XST):
        """This function receives XST messages from the modem
        """
        # Get the current time
        xst_str = "%s," (rospy.Time.now())
        # Append each message fields as a comma-separated string
        for field in msg:
            xst_str += "%s, " % field
        # If the log list is empty, append headers
        if not self.xst_logs:
            self.xst_logs.append("ros_time, version, modem_time, toa_mode, mode, probe_length, bandwidth, carrier, rate_num, src, dest, ack, num_frames_expected, num_frames_sent, packet_type, nbytes, actual_carrier")
        # Append the XST message to the log list
        self.xst_logs.append(xst_str)
        return

    def on_cst(self, msg: CST):
        """This function receives CST messages from the modem
        """
        # Get the current time
        cst_str = "%s," (rospy.Time.now())
        # Append each message fields as a comma-separated string
        for field in msg:
            cst_str += "%s, " % field
        # If the log list is empty, append headers
        if not self.cst_logs:
            self.cst_logs.append("ros_time, version, mode, toa, toa_mode, mfd_peak, mfd_pow, mfd_ratio, mfd_spl, agn, shift_ainp, shift_ain, shift_mfd, shift_p2b, rate_num, src, dest, psk_error, packet_type, num_frames, bad_frames_num, snr_rss, snr_in, snr_out, snr_sym, mse, dqf, dop, noise, carrier, bandwidth, sequence_num, data_rate, num_data_frames, num_bad_data_frames")
        # Append the CST message to the log list
        self.cst_logs.append(cst_str)
        return
