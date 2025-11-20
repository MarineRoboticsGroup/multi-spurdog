# Adding Per-Landmark Statistics to MOOS Alog

## Summary
Following Raymond's suggestion to add per-deckbox metrics tracking to the spurdog_moos_bridge.

## What I've Created

### 1. New ROS Message: `LandmarkStats.msg`
**Location:** `spurdog_acomms/msg/LandmarkStats.msg`

Tracks per-landmark metrics:
- Ping counts (sent, received, success rate)
- Recent performance window (last 10 pings)
- Signal quality (SNR, OWTT, range)
- Health indicators (time since last reply, responsiveness flag)

### 2. ROS Node: `landmark_stats_tracker.py`
**Location:** `spurdog_acomms/src/landmark_stats_tracker.py`

**What it does:**
- Subscribes to `modem/ping_reply`
- Tracks statistics per landmark (L0, L1, etc.)
- Publishes to:
  - `landmark_stats/L0` (individual landmark topics)
  - `landmark_stats/L1`
  - `landmark_stats/all` (aggregate)

**Key features:**
- Automatically detects broken transducers (no replies > 30s)
- Tracks recent success rate (sliding window of last 10 pings)
- Records signal quality from each ping

### 3. Changes Needed in `spurdog_moos_bridge/src/udp2ros.cpp`

Following the pattern from `callbackPingReply`, add:

#### Step 1: Add include at top
```cpp
#include <spurdog_acomms/LandmarkStats.h>
```

#### Step 2: Add subscriber in ROSToUDPPublisher constructor
Around line 626 (after the existing subscribers), add:
```cpp
} else if (topic == "landmark_stats/all") {
    subscriber_ = nh.subscribe<spurdog_acomms::LandmarkStats>(topic, 10, &ROSToUDPPublisher::callbackLandmarkStats, this);
    ROS_INFO("Subscribed to ROS topic: %s", topic.c_str());
```

#### Step 3: Add callback function in private section
After `callbackReceivedSignalStats`, add:
```cpp
void callbackLandmarkStats(const spurdog_acomms::LandmarkStats::ConstPtr &msg)
{
    // Extract fields
    std::string landmark_name = msg->landmark_name;
    int16_t landmark_src = msg->landmark_src;
    uint32_t pings_sent = msg->pings_sent_to_landmark;
    uint32_t replies_received = msg->ping_replies_received;
    float success_rate = msg->success_rate_percent;
    float recent_success = msg->recent_success_rate_percent;
    float last_snr_in = msg->last_snr_in;
    float last_snr_out = msg->last_snr_out;
    float last_range = msg->last_range_meters;
    double last_ping_time = msg->last_successful_ping_time.toSec();
    double time_since_reply = msg->time_since_last_reply.toSec();
    bool is_responsive = msg->is_responsive;
    
    // Construct UDP message for alog
    std::ostringstream oss;
    oss << "<LANDMARK_STATS_" << landmark_name << "=";
    oss << "src=" << landmark_src << ",";
    oss << "pings_sent=" << pings_sent << ",";
    oss << "replies=" << replies_received << ",";
    oss << "success_rate=" << std::fixed << std::setprecision(2) << success_rate << ",";
    oss << "recent_success=" << std::fixed << std::setprecision(2) << recent_success << ",";
    oss << "last_snr_in=" << std::fixed << std::setprecision(2) << last_snr_in << ",";
    oss << "last_snr_out=" << std::fixed << std::setprecision(2) << last_snr_out << ",";
    oss << "last_range=" << std::fixed << std::setprecision(3) << last_range << ",";
    oss << "time_since_reply=" << std::fixed << std::setprecision(2) << time_since_reply << ",";
    oss << "responsive=" << (is_responsive ? "true" : "false");
    oss << ">";
    
    socket_broker.send(oss.str());
}
```

#### Step 4: Add publisher instance in main()
Around line 899 (after existing ROSToUDPPublisher instances), add:
```cpp
ROSToUDPPublisher pub_landmark_stats(nh, "landmark_stats/all", "192.168.10.255", 9105);
```

#### Step 5: Add thread for the publisher
Around line 919 (after existing threads), add:
```cpp
std::thread t9([&]() { pub_landmark_stats.run(); });
```

And update the join section:
```cpp
t9.join();
```

## How to Use

### 1. Add message to CMakeLists.txt
In `spurdog_acomms/CMakeLists.txt`, add to `add_message_files`:
```cmake
add_message_files(
  FILES
  # ... existing messages ...
  LandmarkStats.msg
)
```

### 2. Rebuild workspace
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

### 3. Launch the tracker node
Add to your AUV payload launch file (e.g., `payload_lbl.launch`):
```xml
<node name="landmark_stats_tracker" 
      pkg="spurdog_acomms" 
      type="landmark_stats_tracker.py" 
      output="log">
  <param name="responsiveness_threshold_sec" value="30.0"/>
  <param name="publish_rate_hz" value="1.0"/>
  <param name="sound_speed" value="1500.0"/>
</node>
```

### 4. What appears in the alog

You'll see entries like:
```
<LANDMARK_STATS_L0=src=1,pings_sent=45,replies=42,success_rate=93.33,recent_success=100.00,last_snr_in=15.30,last_snr_out=18.40,last_range=42.350,time_since_reply=1.20,responsive=true>
<LANDMARK_STATS_L1=src=2,pings_sent=45,replies=12,success_rate=26.67,recent_success=0.00,last_snr_in=8.20,last_snr_out=10.10,last_range=38.100,time_since_reply=45.80,responsive=false>
```

**In this example:** L0 is healthy (93% success, responsive), but L1 has a problem (26% success, not responsive for 45s) — likely a broken transducer!

## Post-Experiment Analysis

After the mission, you can parse the alog and plot:
1. Success rate timeline for each landmark
2. Range measurements over time
3. SNR trends
4. Responsiveness gaps (identify when landmarks went silent)

## Useful Metrics at a Glance

**Real-time (during mission):**
- `responsive=false` → immediate alert that a landmark is down
- `recent_success < 50%` → degraded performance
- `time_since_reply > 30s` → landmark silent

**Post-mission:**
- Total success rate per landmark
- Range consistency (detect position drift)
- SNR trends (acoustic environment quality)

---

## Next Steps

Want me to:
1. **Apply these changes to udp2ros.cpp now**?
2. **Add the message to CMakeLists.txt**?
3. **Integrate the tracker node into your payload launch files**?
4. **Create a post-processing script to analyze alog files and plot landmark performance**?

Let me know what you'd like me to do!
