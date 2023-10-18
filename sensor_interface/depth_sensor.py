#!/usr/bin/env python3

import ms5837
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32

class DepthSensor(Node):
    def __init__(self):
        super().__init__("depth_sensor")
        self.publisher = self.create_publisher(Float32, 'rov/depth_sensor', queue_size=10)
        self.sensor = ms5837.MS5837()
        self.logger = self.get_logger() 
        self.sensor_init()
        self.create_timer(0.1, self.pub_sensor)
        
    def sensor_init(self):
        connected = False
        counter = 1
        while not connected and counter <= 10:
            try:
                self.sensor.init()
            except IOError as e:
                self.logger.error(f'{counter} Time: {e}')
            else:
                connected = True
                self.logger.info(f'{counter} Time: depth sensor connected.')
                self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
            counter += 1
            time.sleep(0.1)

        return connected
    
    def pub_sensor(self):
        try:
            self.sensor.read()
            msg = Float32()
            msg.data = self.sensor.depth()
            self.publisher.publish(msg)
        except IOError as e:
            self.logger.error(f'sensor read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    depth_sensor = DepthSensor()
    rclpy.spin(depth_sensor)
    rclpy.shutdown()

if __name__ == "__main__":
    main()