#include <thread>
#include <sstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <iomanip>
#include <ctime>
#include <map>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unordered_set>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "networkclient.hpp"
#include "networkbroker.hpp"
#include <mutex>
#include <chrono>

struct Quaternion
{
    double w, x, y, z;
};

class UDPToROSPublisher
{
public:
    UDPToROSPublisher(ros::NodeHandle &nh, const std::string &ip, int port, int ms_sleep_time, const std::string &messageType, const std::string &topic)
        // ip and port define the UDP socket that gets used
        // messageType is the desired output msg type (in ROS msg format)
        // topic is the topic name
        : socket_client(ip, port), message_type_(messageType), m_thread_sleep_time(ms_sleep_time)
    {
        m_start_time = ros::Time::now(); // Initialize start time
        ROS_INFO("Node start time set to: %u.%09u", m_start_time.sec, m_start_time.nsec);

        // Log when we're initializing the publisher
        ROS_INFO("Initializing UDPToROSPublisher for message type: %s", message_type_.c_str());
        // Initialize a start time
        //m_ros_node_start_time = ros::Time::now();
        // Initialize publishers based on message type
        if (message_type_ == "moosdb")
        {
            publisher_nav_state_ = nh.advertise<geometry_msgs::PoseStamped>("nav_state", 10); // NAV_X, NAV_Y, NAV_DEPTH, NAV_ROLL, NAV_PITCH, NAV_YAW
            publisher_mission_state_ = nh.advertise<std_msgs::String>("mission_state", 10);   // IN_WATER, MISSION_COMPLETE
            //publisher_gps_ = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);                 // NAV_LAT, NAV_LON, HDOP
            // Insteac of publishing the GPS data from NAV_LAT, NAV_LON, HDOP, we can use the NAV_X_GPS and NAV_Y_GPS keys
            publisher_gps_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps", 10); // NAV_X_GPS, NAV_Y_GPS, HDOP, GPS_HAS_LOCK
        }
        else if (message_type_ == "string")
        {
            publisher_string_ = nh.advertise<std_msgs::String>(topic, 10);
        }
        else if (message_type_ == "imu")
        {
            publisher_imu_ = nh.advertise<sensor_msgs::Imu>(topic, 10);
        }
        else if (message_type_ == "dvl_pdx")
        {
            publisher_dvpdx_ = nh.advertise<geometry_msgs::TwistStamped>(topic, 10);
        }
        std::string error = socket_client.open();
        // Log if there was an error opening the socket
        if (!error.empty())
        {
            ROS_ERROR("Error opening socket: %s", error.c_str());
        }
        else
        {
            ROS_INFO("UDP socket opened successfully");
        }
    }

    void run()
    {
        while (ros::ok())
        {
            std::string contents;
            // double len = socket_client.readSentence(contents, "<", ">");  // Read a sentence between '*' and '*'
            double len = socket_client.readSentence(contents, "<", ">");
            if (len > 0)
            { // If a valid sentence was received
                // Then remove the first and last char of the sequence
                contents = contents.erase(0, 1);
                contents = contents.erase(len - 2, 1);
                // Log the contents of the received message
                // ROS_INFO("Received UDP data: %s", contents.c_str());

                // Now process the message based on its type
                if (message_type_ == "moosdb")
                {
                    publishMoosDB(contents);
                }
                else if (message_type_ == "string")
                {
                    publishString(contents);
                }
                else if (message_type_ == "imu")
                {
                    publishImu(contents);
                }
                else if (message_type_ == "dvl_pdx")
                {
                    publishDVPDX(contents);
                }
                else
                {
                    ROS_WARN("Message type not recognized.");
                }
            }
            else
            {
                // If no complete sentence was found
                ROS_WARN("No data received from UDP socket.");
            }

            {
                std::lock_guard<std::mutex> lock(m_ros_lock);
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(m_thread_sleep_time));
            }
        }
    }

private:
    SocketClient socket_client; // Instance of your SocketClient
    // Publishers for MOOSDB messages
    ros::Publisher publisher_mission_state_;
    ros::Publisher publisher_nav_state_;
    ros::Publisher publisher_gps_;
    // Pulishers for other ports
    ros::Publisher publisher_string_;
    ros::Publisher publisher_imu_;
    ros::Publisher publisher_dvpdx_;
    std::string message_type_;
    std::mutex m_ros_lock;
    int m_thread_sleep_time;
    ros::Time m_start_time;

    ros::Time adjustTimestamp(double udp_timestamp)
    {
        // Convert the timestamp to a ros::Time object
        double offset = udp_timestamp - m_start_time.toSec();
        return ros::Time::now() + ros::Duration(offset);
    }

    void publishMoosDB(const std::string &contents)
    {
        // Contents formatted as follows from NetworkBroker.cpp:
        // m_tracked_vars[key] = to_string(MOOSTime()) + "|" + key + "=" + msg.GetString(), then comma separated
        // i.e. "MOOSTIME|NAV_X=1.0,MOOSTIME|NAV_Y=1.0,..."

        // Log the contents of the message
        ROS_INFO("Received MOOSDB Message %s", contents.c_str());

        // Parse the contents string into MOOSDB message fields.
        std::istringstream ss(contents);
        std::string item;
        std::map<std::string, std::string> moosdb;
        std::map<std::string, double> nav_state_buffer;
        std::map<std::string, double> gps_buffer;
        std::string gps_has_lock;
        // Define the sets for message key groupings
        const std::unordered_set<std::string> missionStateKeys = {
            "IN_WATER", "MISSION_COMPLETE"};
        // NAV STATE keys
        const std::unordered_set<std::string> navStateKeys = {"NAV_X", "NAV_Y", "NAV_DEPTH", "NAV_ROLL", "NAV_PITCH", "NAV_YAW", "NAV_HEADING"};
        // const std::unordered_set<std::string> navStateKeys = {
        //     "NAV_DEPTH", "NAV_ROLL", "NAV_PITCH", "NAV_YAW"};
        // const std::unordered_set<std::string> gpsKeys = {
        //     "NAV_LAT", "NAV_LON", "HDOP", "GPS_HAS_LOCK"};
        const std::unordered_set<std::string> gpsKeys = {"NAV_X_GPS", "NAV_Y_GPS", "HDOP"};

        // Parse the string into key-value pairs
        while (std::getline(ss, item, ','))
        {
            std::istringstream ss_item(item);
            std::string timestamp, key, value;
            // Extract the timestamp
            std::getline(ss_item, timestamp, '|');
            // Extract the key and value
            std::getline(ss_item, key, '=');
            std::getline(ss_item, value);
            // Log the parsing
            ROS_INFO("Parsed MOOSDB Message: %s is %s at %s", key.c_str(), value.c_str(), timestamp.c_str());
            // std::getline(ss_item, key, '=');
            // ss_item >> value;
            // moosdb[key] = value;

            // Mission State Data (IN_WATER, MISSION_COMPLETE)
            if (missionStateKeys.find(key) != missionStateKeys.end())
            {
                // Log the mission state value
                ROS_INFO("Mission State - %s: %s", key.c_str(), value.c_str());
                // Publish the mission state
                publishMissionState(key, value);

                // Check for navigation state keys
            }
            else if (navStateKeys.find(key) != navStateKeys.end())
            {
                // ROS_INFO("Received NavState MOOSDB Message - %s: %s", key.c_str(), value.c_str());
                //  Convert the string value to double
                double data = std::stod(value);
                // Add that key-value pair to the nav state buffer
                nav_state_buffer[key] = data;
                // update the buffer timestamp
                nav_state_buffer["timestamp"] = std::stod(timestamp);
                // Debug statement for nav_state_buffer
                //ROS_INFO("NavState buffer contents: ");
                // for (const auto& kv : nav_state_buffer) {
                //      ROS_INFO("  %s = %f", kv.first.c_str(), kv.second);
                //  }
                // Check if the buffer is full
                if (nav_state_buffer.size() == 5)
                {
                    // Log that we're sending the message to ROS
                    ROS_INFO("Nav message full, publishing...");
                    // Pop the navy buffer to the publishNavState function
                    publishNavState(nav_state_buffer);
                    // Clear the buffer
                    nav_state_buffer.clear();
                }
                else
                {
                    continue;
                }

                // GPS Data (GPS_LAT, GPS_LON, HDOP)
            }
            else if (gpsKeys.find(key) != gpsKeys.end())
            {
              // Convert the string value to double
              double data = std::stod(value);
              // Add that key-value pair to the gps buffer
              gps_buffer[key] = data;
              // update the buffer timestamp
              gps_buffer["timestamp"] = std::stod(timestamp);
              // Debug statement for gps_buffer
              ROS_INFO("GPS buffer contents: ");
              for (const auto &kv : gps_buffer)
                {
                  ROS_INFO("  %s = %f", kv.first.c_str(), kv.second);
                }
                // Check if the buffer is full
                if (gps_buffer.size() == 4)
                {
                  // Log that we're sending the message to ROS
                  ROS_INFO("GPS message full, publishing...");
                  // Pop the gps buffer to the publishGPS function
                  publishGPS(gps_buffer);
                  // Clear the buffer
                  gps_buffer.clear();
                }
              }
            }
        }
    }

    void publishString(const std::string &contents)
    {
        std_msgs::String msg;
        msg.data = contents;
        ROS_INFO("Built String message...");
        publisher_string_.publish(msg);
        // {
        //     std::lock_guard<std::mutex> lock(m_ros_lock);
        //     ros::spinOnce();
        // }
    }

    void publishNavState(const std::map<std::string, double> &contents)
    {
        // Convert the buffer rpy to quaternion
        double roll = contents.at("NAV_ROLL");
        //double roll = 0;
        double pitch = contents.at("NAV_PITCH");
        //double pitch = 0;
        double yaw = contents.at("NAV_YAW");
        Quaternion q = rpyToQuaternion(roll, pitch, yaw);

        // Parse the contents string into NavState message fields.
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time(contents.at("timestamp"));
        //msg.header.stamp = adjustTimestamp(contents.at("timestamp"));
        //msg.pose.position.x = contents.at("NAV_X");
        //msg.pose.position.y = contents.at("NAV_Y");
        msg.pose.position.x = 0;
        msg.pose.position.y = 0;
        msg.pose.position.z = contents.at("NAV_DEPTH");
        msg.pose.orientation.x = q.x;
        msg.pose.orientation.y = q.y;
        msg.pose.orientation.z = q.z;
        msg.pose.orientation.w = q.w;
        // Log completion of message
        ROS_INFO("Built PoseStamped message...");
        // Send te message to the ros namespace
        publisher_nav_state_.publish(msg);
        // {
        //     std::lock_guard<std::mutex> lock(m_ros_lock);
        //     ros::spinOnce();
        // }
    }

    void publishMissionState(const std::string &key, std::string &value)
    {
        // Parse the contents string into MissionState message fields.
        std_msgs::String msg;
        msg.data = key + "=" + value;
        // Log completion of message
        ROS_INFO("Built MissionState message...");
        publisher_mission_state_.publish(msg);
        // {
        //     std::lock_guard<std::mutex> lock(m_ros_lock);
        //     ros::spinOnce();
        // }
    }

    void publishGPS(const std::map<std::string, double> &contents)
    {
        sensor_msgs::NavSatFix msg;
        // Set the message header
        msg.header.stamp = ros::Time(contents.at("timestamp"));
        //msg.header.stamp = adjustTimestamp(contents.at("timestamp"));
        // Set the GPS data
        msg.latitude = contents.at("NAV_LAT");
        msg.longitude = contents.at("NAV_LON");
        msg.altitude = 0;
        msg.position_covariance = hdopToCovariance(contents.at("HDOP"), 1.7);
        msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        // Log completion of message
        ROS_INFO("Built NavSatFix message...");
        // Publish the message
        publisher_gps_.publish(msg);
        ROS_INFO("Sent NavSatFix message...");
    }

    void publishImu(const std::string &contents)
    {

        // Parse the contents string into IMU message fields.
        // Assuming contents are formatted as follows (from vNavigatorAHRS.hpp):
        // "[0]timestamp, [1]compass_x, [2]compass_y, [3]compass_z, [4]mag_x, [5]mag_y,
        // [6]mag_z, [7]accel_x, [8]accel_y, [9]accel_z, [10]gyro_x, [11]gyro_y, [12]gyro_z,
        // [13]roll, [14]pitch, [15]yaw, [16]heading, [17]quat_w, [18]quat_x, [19]quat_y, [20]quat_z"
        // std::istringstream ss(contents);
        // std::string item;
        // std::vector<double> imu_data;
        // Improved parsing
        std::string clean_contents = contents;
        if (!clean_contents.empty() && clean_contents.front() == '|')
        {
            clean_contents.erase(0, 1); // Remove the leading '|'
        }

        if (!clean_contents.empty() && clean_contents.back() == '*')
        {
            clean_contents.pop_back(); // Remove the trailing '*'
        }
        // Parse the cleaned-up string into IMU message fields
        sensor_msgs::Imu msg;
        std::istringstream ss(clean_contents);
        std::string item;
        // std::vector<double> imu_data;
        // while (std::getLine(ss, item, ','))
        // {
        // Modified parsing structure- deprecated
        // std::getline(ss, item, ',');
        //     uint64_t time_since_epoch = 0;
        // try {
        //      // Print the raw data in case there are hidden characters
        //     // ROS_INFO("Raw Unparsed Timestamp (Hex): ");
        //     // for (char c : item) {
        //     //     ROS_INFO("%X ", (unsigned char)c);
        //     // }
    
        //     // Attempt to parse as double (in case of scientific notation)
        //     double timestamp_double = std::stod(item);
        //     uint64_t timestamp_int = static_cast<uint64_t>(timestamp_double); // Truncate the fractional part
    
        //     // ROS_INFO("Parsed 64-bit Timestamp (from double): %llu", timestamp_int);

        //     time_since_epoch = timestamp_int;

        // } catch (const std::invalid_argument &e) {
        //     ROS_ERROR("Error parsing timestamp: %s", e.what());
        //     return;
        // } catch (const std::out_of_range &e) {
        //     ROS_ERROR("Timestamp out of range: %s", e.what());
        //     return;
        // }
        
        // // Deprecated modified time parsing
        //ROS_INFO("Time since epoch: %u", time_since_epoch);
        // ros::Time current;
        // ros::Time timestamp;
        // current = ros::Time::now();
        // timestamp = current;
        // timestamp.sec = (current.sec + time_since_epoch / 1e9);       // Convert nanoseconds to seconds
        // timestamp.nsec = (current.nsec + time_since_epoch % static_cast<uint64_t>(1e9));
        // ROS_INFO("Parsed Timestamp: %u.%09u", timestamp.sec, timestamp.nsec);

        // Parse the remaining IMU data
        std::vector<double> imu_data;
        while (std::getline(ss, item, ',')) {
            try {
                imu_data.push_back(std::stod(item)); // Convert strings to double
            } catch (const std::invalid_argument &e) {
                ROS_ERROR("Error parsing IMU data: %s", e.what());
                return;
            } catch (const std::out_of_range &e) {
                ROS_ERROR("IMU data out of range: %s", e.what());
                return;
            }
        }
        // Print the IMU Data
        // std::stringstream imu_output;
        // imu_output << "Parsed IMU Data: ";
        // for (const auto &value : imu_data)
        // {
        //     imu_output << value << " "; // Append each IMU data value to the stringstream
        // }
        // ROS_INFO_STREAM(imu_output.str());

        // ROS_INFO("Separated the IMU Message into fields");
        if (imu_data.size() >= 21)
        { // Check if we have enough data
            // Set the message header
            // convert the timestamp to a ros::Time object
            msg.header.stamp = adjustTimestamp(imu_data[0]);
            // Set the orientation
            msg.orientation.x = imu_data[18];
            msg.orientation.y = imu_data[19];
            msg.orientation.z = imu_data[20];
            msg.orientation.w = imu_data[17];
            // Set the angular velocity
            msg.angular_velocity.x = imu_data[10];
            msg.angular_velocity.y = imu_data[11];
            msg.angular_velocity.z = imu_data[12];
            // Set the linear acceleration
            msg.linear_acceleration.x = imu_data[7];
            msg.linear_acceleration.y = imu_data[8];
            msg.linear_acceleration.z = imu_data[9];
            // Log completion of message
            //ROS_INFO("Built IMU message...");
            publisher_imu_.publish(msg);
            ROS_INFO("Sent IMU message...");
            {
                std::lock_guard<std::mutex> lock(m_ros_lock);
                ros::spinOnce();
            }
        }
        else
        {
            ROS_WARN("IMU data does not have the required 21 fields. Data size: %zu", imu_data.size());
        }
    }

    void publishDVPDX(const std::string &contents)
    {
        geometry_msgs::TwistStamped msg;

        // Parse the contents string into TwistStamped message fields.
        // Assuming contents are formatted as follows (from Tracker650Driver.hpp):
        // "[0]timesinceboot, [1]duT, [2]angleDeltaRoll, [3]angleDeltaPitch, [4]angleDeltaYaw,
        // [5]deltaX, [6]deltaY, [7]deltaZ, [8]latestConfidence, [9]mode,
        // [10]pitch, [11]roll, [12]yaw, [13]standoff, [14]valid"
        std::istringstream ss(contents);
        std::string item;
        std::vector<double> dvl_data;

        // If an item is a number convert it to a double, otherwise leave it as a string
        while (std::getline(ss, item, ','))
        {
            dvl_data.push_back(std::stod(item)); // Convert strings to double
        }

        dvl_data[1] = dvl_data[1] / 1000000; // Convert from microseconds to seconds
        // Convert deltas to linear velocities (m/s)
        double vx = dvl_data[5] / dvl_data[1];
        double vy = dvl_data[6] / dvl_data[1];
        double vz = dvl_data[7] / dvl_data[1];
        // Convert the angles to rad (rad)
        double roll = dvl_data[11] * (M_PI / 180);
        double pitch = dvl_data[10] * (M_PI / 180);
        double yaw = dvl_data[12] * (M_PI / 180);
        // Convert the angular deltas to angular velocities (rad/s)
        double vroll = (dvl_data[2] / dvl_data[1]) * (M_PI / 180);
        double vpitch = (dvl_data[3] / dvl_data[1]) * (M_PI / 180);
        double vyaw = (dvl_data[4] / dvl_data[1]) * (M_PI / 180);

        // Convert the angular velocities to the body frame
        double wx = vroll + vpitch * sin(roll) * tan(pitch) + vyaw * cos(roll) * tan(pitch);
        double wy = vpitch * cos(roll) - vyaw * sin(roll);
        double wz = vpitch * sin(roll) / cos(pitch) + vyaw * cos(roll) / cos(pitch);

        // Construct message
        if (dvl_data.size() >= 15)
        { // Check if we have enough data
            // Set the message header
            msg.header.stamp = adjustTimestamp(dvl_data[0]);
            // Set the linear velocity
            msg.twist.linear.x = vx;
            msg.twist.linear.y = vy;
            msg.twist.linear.z = vz;
            // Set the angular velocity
            msg.twist.angular.x = wx;
            msg.twist.angular.y = wy;
            msg.twist.angular.z = wz;
        }
        // Log completion of message
        ROS_INFO("Built TwistStamped message...");
        publisher_dvpdx_.publish(msg);
    }

    Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
    {
        // Convert roll, pitch, yaw to radians
        roll = roll * (M_PI / 180);
        pitch = pitch * (M_PI / 180);
        yaw = yaw * (M_PI / 180);
        // Calculate the quaternion components
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    boost::array<double, 9> hdopToCovariance(double hdop, double sigma_h)
    {
        // Variances
        double var_x = pow((hdop * sigma_h), 2);
        double var_y = pow((hdop * sigma_h), 2);
        double var_z = pow((hdop * sigma_h * 1.7), 2);
        // Covariances
        boost::array<double, 9> covariance = {0.0};
        covariance[0] = var_x;
        covariance[4] = var_y;
        covariance[8] = var_z;
        return covariance;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp2ros");
    ros::NodeHandle nh;
    //ros::Time m_start_time = ros::Time::now();

    // Create instances for different ports and message types
    int moos_sleep_time_ms = 75;
    int ahrs_sleep_time_ms = 5;
    int dvl_sleep_time_ms = 50;
    UDPToROSPublisher pub_moosdb(nh, "192.168.10.255", 9100, moos_sleep_time_ms, "moosdb", ""); // Note: this xmits several ros topics
    UDPToROSPublisher pub_nav_ahrs(nh, "192.168.10.255", 9101, ahrs_sleep_time_ms, "imu", "navigator_ahrs");
    //UDPToROSPublisher pub_dvl_pdx(nh, "192.168.10.255", 9102, "dvl_pdx", "dvl_pdx");
    // Add more publishers for other message types here

    std::thread t1([&]()
                   { pub_moosdb.run(); });
    std::thread t2([&]()
                   { pub_nav_ahrs.run(); });
    // std::thread t3([&]()
    //                { pub_dvl_pdx.run(); });
    // Add more threads for other publishers here

    // ros::spin();

    t1.join();
    t2.join();
    //t3.join();
    // Join other threads here

    return 0;
}