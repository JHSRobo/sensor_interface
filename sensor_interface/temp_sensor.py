#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
import board
import adafruit_ahtx0

class TempSensor(Node):
    def __init__(self):
        super().__init__("temp_sensor")

        qos_profile = QoSProfile(depth=10)

        # Create publishers for each sensor on the board
        # A Hygrometer measures humidity
        self.hygrometer_pub = self.create_publisher(Float32, 'hygrometer', qos_profile)
        self.thermometer_pub = self.create_publisher(Float32, 'thermometer', qos_profile)

        self.logger = self.get_logger()
        self.connected = False

        self.sensor_init()
        
        if self.connected: self.create_timer(0.01, self.pub_sensors)

    def sensor_init(self):

        # Try to connect to sensor.
        try:
            self.i2c = board.I2C()
            self.sensor = adafruit_ahtx0.AHTx0(self.i2c)
        except:
            self.logger.warn("Cannot connect to AHT20. Ignore this if Temp/Humidity sensor is unplugged")
        else:
            self.connected = True
    
    def pub_sensors(self):

        # Publish Temperature
        temperature_msg = self.create_float_msg(self.sensor.temperature)
        if temperature_msg is not None: self.thermometer_pub.publish(temperature_msg)

        # Publish Humidity
        humidity_msg = self.create_float_msg(self.sensor.relative_humidity)
        if humidity_msg is not None: 
            self.hygrometer_pub.publish(humidity_msg)


    def create_float_msg(self, measurement): 
        float_msg = Float32()
        if measurement is None:
            return None
        float_msg.data = round(float(measurement), 2)

        return float_msg


def main(args=None):
    rclpy.init(args=args)
    temp_sensor = TempSensor()
    rclpy.spin(temp_sensor)
    temp_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()