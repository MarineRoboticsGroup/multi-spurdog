#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import BatteryState
from ros_acomms_msgs.msg import ModemSensorData


def get_float_from_string_with_units(input_string: str) -> float:
    return float(''.join(c for c in input_string if (c.isdigit() or c == '.' or c == '-')))


class ModemSensorDataNode(object):
    fields = {'pwramp.temperature_degC': 'pwramp_temperature',
              'pwramp.vbat': 'pwramp_vbat',
              'pwramp.vtransmit': 'pwramp_vtransmit',
              'hwd.batt_v': 'modem_batt_v',
              'hwd.temp': 'modem_temp'}

    def __init__(self):
        rospy.init_node('modem_sensor_data')

        self.query_interval_seconds = rospy.get_param('~query_interval_seconds', 30)
        self.enable_battery_query = rospy.get_param('~enable_battery_query', default=False)
        self.battery_state_topic = rospy.get_param('~battery_state_topic', 'battery_state')
        minimum_interval = 5 if not self.query_battery else 30
        if self.query_interval_seconds < minimum_interval:
            rospy.logwarn(f'Query interval must be at least {minimum_interval} seconds.  Setting to {minimum_interval}')
            self.query_interval_seconds = minimum_interval

        self.modem_sensor_data_publisher = rospy.Publisher('modem_sensor_data', ModemSensorData, queue_size=10)
        if self.query_battery:
            self.battery_state_publisher = rospy.Publisher(self.battery_state_topic, BatteryState, queue_size=10)
        self.nmea_publisher = rospy.Publisher('nmea_to_modem', String, queue_size=10)

        self.value_dict = {}
        self.battery_percent = 0
        self.battery_voltage_mV = 0
        self.battery_current_mA = 0
        self.battery_remaining_capacity_mAh = 0

        rospy.Subscriber('nmea_from_modem', String, self.on_nmea_subscriber)

        rospy.loginfo("Modem sensor data node running")

        self.spin()

    def spin(self):
        while not rospy.is_shutdown():
            self.query_modem()
            rospy.sleep(2)
            if self.enable_battery_query:
                self.query_battery()
            rospy.sleep(2)
            self.publish_data_message()
            if self.enable_battery_query:
                self.publish_battery_state_message()
            rospy.sleep(self.query_interval_seconds - 4)

    def query_modem(self):
        for cfg_param in self.fields.keys():
            nmea_string = f"$CCCFQ,{cfg_param}"
            msg = String(data=nmea_string)
            self.nmea_publisher.publish(msg)

    def query_battery(self):
        nmea_string = "$CCBAT,0"
        msg = String(data=nmea_string)
        self.nmea_publisher.publish(msg)

    def on_nmea_subscriber(self, msg: String) -> None:
        nmea_string = msg.data
        try:
            if "CACFG" in nmea_string:
                msg_parts = nmea_string.split(',')
                cfg_param = msg_parts[1]
                if cfg_param in self.fields:
                    value = float(msg_parts[2].split('*')[0])
                    self.value_dict[self.fields[cfg_param]] = value
            elif "CABAT" in nmea_string:
                msg_parts = nmea_string.split(',')
                self.battery_percent = get_float_from_string_with_units(msg_parts[1])
                self.battery_voltage_mV = get_float_from_string_with_units(msg_parts[2])
                self.battery_current_mA = get_float_from_string_with_units(msg_parts[3])
                self.battery_remaining_capacity_mAh = get_float_from_string_with_units(msg_parts[5])
        except Exception as e:
            rospy.logerr(f"Error parsing NMEA.  Message was {nmea_string.strip()}, error is: {e}")

    def publish_data_message(self):
        sensor_data_msg = ModemSensorData(**self.value_dict)
        sensor_data_msg.header = Header(stamp=rospy.get_rostime())
        self.modem_sensor_data_publisher.publish(sensor_data_msg)

    def publish_battery_state_message(self):
        percent_fraction = self.battery_percent / 100
        battery_voltage_V = self.battery_voltage_mV / 1000
        battery_current_A = self.battery_current_mA / 1000
        battery_remaining_capacity_Ah = self.battery_remaining_capacity_mAh / 1000
        battery_capacity_Ah = battery_remaining_capacity_Ah / (percent_fraction + sys.float_info.epsilon)
        status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if battery_current_A > 0 else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if battery_voltage_V <= 0.0001:
            battery_present = False
        else:
            battery_present = True

        battery_state_msg = BatteryState(header=Header(stamp=rospy.get_rostime()),
                                         voltage=battery_voltage_V,
                                         current=battery_current_A,
                                         charge=battery_remaining_capacity_Ah,
                                         capacity=battery_capacity_Ah,
                                         design_capacity=battery_capacity_Ah,
                                         percentage=percent_fraction,
                                         power_supply_status=status,
                                         power_supply_health=BatteryState.POWER_SUPPLY_HEALTH_GOOD,
                                         power_supply_technology=BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
                                         present=battery_present)

        self.battery_state_publisher.publish(battery_state_msg)

if __name__ == '__main__':
    try:
        node = ModemSensorDataNode()  # This spins.
        rospy.loginfo("ModemSensorDataNode shutdown")
    except rospy.ROSInterruptException:
        rospy.loginfo("ModemSensorDataNode shutdown (interrupt)")
