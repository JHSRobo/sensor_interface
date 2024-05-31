#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
import board
import adafruit_sht31d

class TempSensor(Node):
    def __init__(self):
        super().__init__("temp_sensor")

        self.qos_profile = QoSProfile(depth=10)

        self.logger = self.get_logger()
        self.connected = False

        self.sensor_init()
        
        if self.connected: 
            self.create_timer(0.01, self.pub_sensors)
            self.create_timer(0.1, self.update_parameters)

            # Define the parameter
            slider_bounds = FloatingPointRange()
            slider_bounds.from_value = -3.0
            slider_bounds.to_value = 3.0
            slider_bounds.step = 1.0
            slider_descriptor = ParameterDescriptor(floating_point_range = [slider_bounds])
            self.declare_parameter('temperature_offset', 0.0, slider_descriptor)

            self.offset = 0


    def update_parameters(self):
        self.offset = self.get_parameter('temperature_offset').value

    def sensor_init(self):

        # Try to connect to sensor.
        try:
            self.i2c = board.I2C()
            self.sensor = adafruit_sht31d.SHT31D(self.i2c)
        except:
            self.logger.warn("Cannot connect to SHT31D. Ignore this if external Temp/Humidity sensor is unplugged")
            exit()
        else:
            self.connected = True
            # Create publishers for each sensor on the board
            # A Hygrometer measures humidity
            self.hygrometer_pub = self.create_publisher(Float32, 'outer_hygrometer', self.qos_profile)
            self.thermometer_pub = self.create_publisher(Float32, 'outer_thermometer', self.qos_profile)
    
    def pub_sensors(self):

        # Publish Temperature
        temperature_msg = self.create_float_msg(self.sensor.temperature + self.offset)
        if temperature_msg is not None: self.thermometer_pub.publish(temperature_msg)

        # Publish Humidity
        humidity_msg = self.create_float_msg(self.sensor.relative_humidity + self.offset)
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
