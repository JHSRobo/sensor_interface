import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from core.msg import ThrusterVoltage, ThrusterCurrent

import board, busio
from adafruit_ads1x15 import ADS1015, AnalogIn, ads1x15

class ThrustersMonitorNode(Node):
    def __init__(self):
        super().__init__("thrusters_monitor")

        self.log = self.get_logger()
        self.connected = False

        self.scalar_48V = 19.8
        self.scalar_12V = 5.273
        self.scalar_3V3 = 2
        self.scalar_current = 6.061

        self.sensor_init()

        if self.connected:
            self.log.info("init")
            self.voltage_pub = self.create_publisher(ThrusterVoltage, 'thruster_voltage', 10)
            self.current_pub = self.create_publisher(ThrusterCurrent, 'thruster_current', 10)
            self.create_timer(0.01, self.pub_sensor) 


    def sensor_init(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)

        self.voltage_channels_ads = ADS1015(self.i2c, address=0x4A)
        self.voltage_channels = [AnalogIn(self.voltage_channels_ads, ads1x15.Pin.A0), \
                                 AnalogIn(self.voltage_channels_ads, ads1x15.Pin.A1), \
                                 AnalogIn(self.voltage_channels_ads, ads1x15.Pin.A2), \
                                 AnalogIn(self.voltage_channels_ads, ads1x15.Pin.A3)]

        self.left_thrusters_ads = ADS1015(self.i2c, address=0x48)
        self.left_thrusters = [AnalogIn(self.left_thrusters_ads, ads1x15.Pin.A0), \
                               AnalogIn(self.left_thrusters_ads, ads1x15.Pin.A1), \
                               AnalogIn(self.left_thrusters_ads, ads1x15.Pin.A2), \
                               AnalogIn(self.left_thrusters_ads, ads1x15.Pin.A3)]


        self.right_thrusters_ads = ADS1015(self.i2c, address=0x49)
        self.right_thrusters = [AnalogIn(self.right_thrusters_ads, ads1x15.Pin.A0), \
                               AnalogIn(self.right_thrusters_ads, ads1x15.Pin.A1), \
                               AnalogIn(self.right_thrusters_ads, ads1x15.Pin.A2), \
                               AnalogIn(self.right_thrusters_ads, ads1x15.Pin.A3)]

        self.connected = True

    def pub_sensor(self):
        self.pub_volt()
        self.pub_current()

    def pub_volt(self):
        msg = ThrusterVoltage()
        msg.v48v = self.voltage_channels[0].voltage * self.scalar_48V
        msg.v12v_left = self.voltage_channels[1].voltage * self.scalar_12V
        msg.v12v_right = self.voltage_channels[2].voltage * self.scalar_12V
        msg.v3v3 = self.voltage_channels[3].voltage * self.scalar_3V3
        self.voltage_pub.publish(msg)

    def pub_current(self):
        # Thruster Configuration Order
        # 1 2
        # 5 6
        # 7 8
        # 3 4

        msg = ThrusterCurrent()
        msg.thruster1 = self.left_thrusters[0].voltage * self.scalar_current
        msg.thruster2 = self.right_thrusters[0].voltage * self.scalar_current
        msg.thruster3 = self.left_thrusters[3].voltage * self.scalar_current
        msg.thruster4 = self.right_thrusters[3].voltage * self.scalar_current
        msg.thruster5 = self.left_thrusters[1].voltage * self.scalar_current
        msg.thruster6 = self.right_thrusters[1].voltage * self.scalar_current
        msg.thruster7 = self.left_thrusters[2].voltage * self.scalar_current
        msg.thruster8 = self.right_thrusters[2].voltage * self.scalar_current
        self.current_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    thrusters_monitor = ThrustersMonitorNode()
    rclpy.spin(thrusters_monitor)
    thrusters_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
