#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO

class LeakSensor(Node):
    def __init__(self):
        super().__init__("leak_sensor")

        self.logger = self.get_logger()

        # Set up the board and declare GPIO 27 as an input pin
        RPi.GPIO.setmode(RPi.GPIO.BCM)
        RPi.GPIO.setwarnings(False)
        RPi.GPIO.setup(27, RPi.GPIO.IN)
        
        self.create_timer(0.25, self.check_leaks)

        self.declare_parameter("leak_detection", True)
    
    def check_leaks(self):
        if RPi.GPIO.input(27):
            if self.get_parameter("leak_detection").value:
                self.logger.fatal("WATER IN THE MEH")

def main(args=None):
    rclpy.init(args=args)
    leak_sensor = LeakSensor()
    rclpy.spin(leak_sensor)
    leak_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()