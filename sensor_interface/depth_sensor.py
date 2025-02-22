#!/usr/bin/env python3

from core_lib import ms5837
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


class DepthSensor(Node):
    def __init__(self):
        super().__init__("depth_sensor")
        self.qos_profile = QoSProfile(depth=10)
        self.sensor = ms5837.MS5837_02BA()
        self.log = self.get_logger()
        self.connected = False


        self.sensor_init()

        self.create_timer(0.05, self.pub_sensor)

    def sensor_init(self):

        # Try to connect to sensor.
        try:
            self.sensor.init()
            self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
        except:
            self.log.warn("Cannot connect to MS5837. Ignore this if depth sensor is unplugged")
            exit()
        else:
            self.connected = True
            self.temp_publisher = self.create_publisher(Float32, 'outer_temp_sensor', self.qos_profile)
            self.depth_publisher = self.create_publisher(Float32, 'depth_sensor', self.qos_profile)

            self.create_timer(0.05, self.update_parameters)

            # Define the parameter
            slider_bounds = FloatingPointRange()
            slider_bounds.from_value = -6.0
            slider_bounds.to_value = 6.0
            slider_bounds.step = 0.05
            slider_descriptor = ParameterDescriptor(floating_point_range = [slider_bounds])
            self.declare_parameter('temperature_offset', 0.0, slider_descriptor)

            self.offset = 0
            
            # Create Publisher only if sensor is connected

    def update_parameters(self):
        self.offset = self.get_parameter('temperature_offset').value
    
    def pub_sensor(self):
        if self.connected:
            depth_msg = Float32()
            temp_msg = Float32()
            try: 
                self.sensor.read()
                depth_msg.data = self.sensor.depth()
                temp_msg.data = self.sensor.temperature() + self.offset
            except: pass
            else:
                self.depth_publisher.publish(depth_msg)
                self.temp_publisher.publish(temp_msg)
            

def main(args=None):
    rclpy.init(args=args)
    depth_sensor = DepthSensor()
    rclpy.spin(depth_sensor)
    depth_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
