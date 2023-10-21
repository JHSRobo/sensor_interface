#!/usr/bin/env python3

from core_lib import ms5837
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

class DepthSensor(Node):
    def __init__(self):
        super().__init__("depth_sensor")
        self.qos_profile = QoSProfile(depth=10)
        self.sensor = ms5837.MS5837()
        self.logger = self.get_logger()
        self.connected = False

        self.sensor_init()

        if self.connected: self.create_timer(0.05, self.pub_sensor)

    def sensor_init(self):

        # Try to connect to sensor.
        try:
            self.sensor.init()
            self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
        except:
            self.logger.warn("Cannot connect to MS5837. Ignore this if depth sensor is unplugged")
        else:
            self.connected = True
            
            # Create Publisher only if sensor is connected
            self.publisher = self.create_publisher(Float32, 'depth_sensor', self.qos_profile)
    
    def pub_sensor(self):
        try:
            self.sensor.read()
            msg = Float32()
            msg.data = self.sensor.depth()
            self.publisher.publish(msg)
        except IOError as e:
            self.logger.error(f'Depth sensor read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    depth_sensor = DepthSensor()
    rclpy.spin(depth_sensor)
    depth_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()