#!/usr/bin/env python3
"""
Landmark Statistics Tracker
Monitors ping replies from landmarks and publishes per-landmark performance metrics.
This helps detect broken transducers or communication issues in real-time.
"""
import rospy
from collections import defaultdict, deque
from ros_acomms_msgs.msg import PingReply
from spurdog_acomms.msg import LandmarkStats
from std_msgs.msg import String
from spurdog_acomms_utils.param_utils import get_namespace_param


class LandmarkStatsTracker:
    def __init__(self):
        rospy.init_node('landmark_stats_tracker', anonymous=True)
        
        # Load landmark configuration
        self.landmarks = get_namespace_param("landmarks", {}, warn_if_missing=True)
        
        # Map landmark src addresses to names
        self.src_to_name = {}
        for name, config in self.landmarks.items():
            if isinstance(config, dict) and 'src' in config:
                self.src_to_name[config['src']] = name
            elif isinstance(config, list) and len(config) == 2:
                # Legacy format: [src, pos]
                self.src_to_name[config[0]] = name
        
        rospy.loginfo(f"Tracking landmarks: {self.src_to_name}")
        
        # Statistics storage
        self.stats = defaultdict(lambda: {
            'pings_sent': 0,
            'replies_received': 0,
            'recent_window': deque(maxlen=10),  # Track last 10 pings
            'last_snr_in': 0.0,
            'last_snr_out': 0.0,
            'last_owtt': 0.0,
            'last_range': 0.0,
            'last_success_time': None,
            'was_unresponsive': False,  # Track state to avoid repeated warnings
        })
        
        # Parameters
        self.responsiveness_threshold = rospy.get_param("~responsiveness_threshold_sec", 60.0)
        self.publish_rate = rospy.get_param("~publish_rate_hz", 1.0)
        self.sound_speed = rospy.get_param("~sound_speed", 1500.0)  # m/s
        
        # Subscribers
        self.ping_reply_sub = rospy.Subscriber(
            "modem/ping_reply", 
            PingReply, 
            self.on_ping_reply
        )
        
        # Publishers (one per landmark for easy alog filtering)
        self.landmark_pubs = {}
        for src, name in self.src_to_name.items():
            topic = f"landmark_stats/{name}"
            self.landmark_pubs[src] = rospy.Publisher(topic, LandmarkStats, queue_size=10)
        
        # Also publish aggregate stats
        self.stats_pub = rospy.Publisher("landmark_stats/all", LandmarkStats, queue_size=10)
        
        # LED command publisher for warnings
        self.led_pub = rospy.Publisher("led_command", String, queue_size=10)
        
        rospy.loginfo("Landmark stats tracker initialized")
    
    def on_ping_reply(self, msg):
        """Process incoming ping replies."""
        src = msg.src
        
        if src not in self.src_to_name:
            rospy.logwarn_throttle(10, f"Received ping from unknown src={src}")
            return
        
        name = self.src_to_name[src]
        stat = self.stats[src]
        
        # Update statistics
        stat['replies_received'] += 1
        stat['recent_window'].append(1)  # 1 = success
        stat['last_snr_in'] = msg.cst.snr_in
        stat['last_snr_out'] = msg.cst.snr_out
        stat['last_owtt'] = msg.owtt
        stat['last_range'] = msg.owtt * self.sound_speed
        stat['last_success_time'] = rospy.Time.now()
        
        rospy.logdebug(f"Ping reply from {name}: range={stat['last_range']:.2f}m, SNR_in={stat['last_snr_in']:.1f}dB")
    
    def record_ping_timeout(self, landmark_src):
        """Record a ping timeout (called from external node or via topic)."""
        if landmark_src in self.stats:
            self.stats[landmark_src]['pings_sent'] += 1
            self.stats[landmark_src]['recent_window'].append(0)  # 0 = failure
    
    def publish_stats(self):
        """Publish current statistics for all landmarks."""
        now = rospy.Time.now()
        
        for src, name in self.src_to_name.items():
            stat = self.stats[src]
            
            msg = LandmarkStats()
            msg.header.stamp = now
            msg.landmark_name = name
            msg.landmark_src = src
            
            # Overall stats
            total_pings = stat['pings_sent'] + stat['replies_received']
            msg.pings_sent_to_landmark = total_pings
            msg.ping_replies_received = stat['replies_received']
            msg.success_rate_percent = (
                100.0 * stat['replies_received'] / total_pings 
                if total_pings > 0 else 0.0
            )
            
            # Recent performance
            recent = stat['recent_window']
            msg.recent_window_size = len(recent)
            msg.recent_replies_received = sum(recent)
            msg.recent_success_rate_percent = (
                100.0 * sum(recent) / len(recent) 
                if len(recent) > 0 else 0.0
            )
            
            # Signal quality
            msg.last_snr_in = stat['last_snr_in']
            msg.last_snr_out = stat['last_snr_out']
            msg.last_owtt = stat['last_owtt']
            msg.last_range_meters = stat['last_range']
            
            if stat['last_success_time']:
                msg.last_successful_ping_time = stat['last_success_time']
                time_since = (now - stat['last_success_time']).to_sec()
                msg.time_since_last_reply = rospy.Duration(time_since)
                msg.is_responsive = (time_since < self.responsiveness_threshold)
                
                # Check if landmark became unresponsive
                if not msg.is_responsive and not stat['was_unresponsive']:
                    self.send_unresponsive_warning(name, time_since)
                    stat['was_unresponsive'] = True
                    rospy.logwarn(f"Landmark {name} is UNRESPONSIVE (no reply for {time_since:.1f}s)")
                # Reset warning state when responsive again
                elif msg.is_responsive and stat['was_unresponsive']:
                    stat['was_unresponsive'] = False
                    rospy.loginfo(f"Landmark {name} is responsive again")
            else:
                msg.is_responsive = False
            
            # Publish
            self.landmark_pubs[src].publish(msg)
            self.stats_pub.publish(msg)  # Also publish to aggregate topic
    
    def send_unresponsive_warning(self, landmark_name, time_since):
        """Send LED warning when landmark becomes unresponsive."""
        # Flash red 3 times over 5 seconds: each flash is ~1.67s
        # Pattern format: [R.G.B.A]:duration
        # 3 flashes in 5 seconds = pattern repeats, 1 cycle total
        led_cmd = 'priority=3,pattern=([255.0.0.0]:1.67),cycles=3'
        
        led_msg = String()
        led_msg.data = led_cmd
        self.led_pub.publish(led_msg)
        
        rospy.logwarn(f"Sent LED warning for unresponsive landmark {landmark_name} (no reply for {time_since:.1f}s)")
    
    def run(self):
        """Main loop."""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            self.publish_stats()
            rate.sleep()


if __name__ == '__main__':
    try:
        tracker = LandmarkStatsTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
