#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from core.msg import Channel, Channels
import board
import adafruit_ina3221

class ElectronicSensorNode(Node):
    def __init__(self):
        super().__init__("electronics_sensor")

        self.qos_profile = QoSProfile(depth=10)
        
        self.log = self.get_logger()
        self.connected = False

        self.sensor_init()

        if self.connected:
            self.electronics_pub = self.create_publisher(Channels, 'electronics_data', self.qos_profile)
            # Set the shunt resistance on each channel to be 0.04 ohms
            for i in range(3):
                self.ina[i].shunt_resistance = 0.04
            self.create_timer(0.01, self.pub_sensor) 

    def sensor_init(self):
        # Try to connect to sensor
        try: 
            self.i2c = board.I2C()
            self.ina = adafruit_ina3221.INA3221(self.i2c)
        except:
            self.log.warn("Cannot connect to INA3221. Ignore this if SuperHat is not connected.")
            exit()
        else:
            self.connected = True

    def pub_sensor(self):
        msg = Channels()
        # Channel 1
        msg.channel_1.bus_voltage = self.ina[0].bus_voltage
        msg.channel_1.shunt_voltage = self.ina[0].shunt_voltage
        msg.channel_1.current = self.ina[0].current_amps
        # Channel 2 
        msg.channel_2.bus_voltage = self.ina[1].bus_voltage
        msg.channel_2.shunt_voltage = self.ina[1].shunt_voltage
        msg.channel_2.current = self.ina[1].current_amps
        # Channel 3 
        msg.channel_3.bus_voltage = self.ina[2].bus_voltage
        msg.channel_3.shunt_voltage = self.ina[2].shunt_voltage
        msg.channel_3.current = self.ina[2].current_amps

        self.electronics_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    electronics_node = ElectronicSensorNode()
    rclpy.spin(electronics_node)
    electronics_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
