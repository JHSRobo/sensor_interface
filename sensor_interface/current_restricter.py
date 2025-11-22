import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from core.msg import Thrusterstats, Thrusterstat
import board

i2c = busio.I2C(board.SCL, board.SDA)

left_thrusters = ADS.ADS1015(i2c, address=0x48)
right_thrusters = ADS.ADS1015(i2c, address=0x49)
voltage_channels = ADS.ADS1015(i2c, address=0x4A)

ads_pins = [ADS.P0, ADS.P1, ADS.P2, ADS.P3]

SHUNT_VALUE = 0.0033
AMP_FACTOR = 50

VOLTAGE_GAINS = {
    ADS.P0: 19.8,  # expects ~48V
    ADS.P1: 5.273,  # expects ~12V
    ADS.P2: 2.0,   # expects ~5V
    ADS.P3: 2.0,   # expects ~3.3V
}


class CurrentRestricterNode(Node):
    def __init__(self):
        super().__init__("current_restricter")
        
        self.log = self.get_logger()
        self.connected = False

        self.sensor_init()

        if self.connected:
            self.electronics_pub = self.create_publisher(Thrusterstats, 'thruster_stats', self.qos_profile)

            self.create_timer(0.01, self.pub_sensor) 

    def read_amps(self):
        readings = []
        for name, ads in [("L", left_thrusters), ("R", right_thrusters)]:
            for i, pin in enumerate(ads_pins):
                chan = AnalogIn(ads, pin)
                current = chan.voltage / (SHUNT_VALUE * AMP_FACTOR)
                readings.append(current)
        
        return readings

    def read_volts(self):
        readings = []
        for i, pin in enumerate(ads_pins):
            chan = AnalogIn(voltage_channels, pin)
            voltage = chan.voltage * VOLTAGE_GAINS[pin]
            readings[f"V{i}"] = voltage
        return readings

    def pub_sensor(self):
        msg = Thruster_stats()

        amps = self.read_amps()
        volts = self.read_volts()

        msg.thruster1.current = amps[0]
        msg.thruster1.volts = volts[0]

        msg.thruster2.current = amps[0]
        msg.thruster2.volts = volts[0]

        msg.thruster3.current = amps[0]
        msg.thruster3.volts = volts[0]

        msg.thruster4.current = amps[0]
        msg.thruster4.volts = volts[0]

        msg.thruster5.current = amps[0]
        msg.thruster5.volts = volts[0]

        msg.thruster6.current = amps[0]
        msg.thruster6.volts = volts[0]

        msg.thruster7.current = amps[0]
        msg.thruster7.volts = volts[0]

        msg.thruster8.current = amps[0]
        msg.thruster8.volts = volts[0]

        self.electronics_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    current_restricter = CurrentRestricterNode()
    rclpy.spin(current_restricter)
    current_restricter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
