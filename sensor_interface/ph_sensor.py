import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PHSensorNode(Node): # Create a class that goes from the Node class
    def __init__(self):
        super().__init__('ph_sensor_node')
        self.publisher_voltage = self.create_publisher(Float32, 'ph_sensor/voltage', 10) # Create a publisher to publish the voltage value
        self.publisher_ph = self.create_publisher(Float32, 'ph_sensor/ph', 10) # Create a publisher to publish the pH value
        self.timer = self.create_timer(1, self.read_sensor) # Create a timer to read the sensor every second

        i2c = busio.I2C(board.SCL, board.SDA) # Initialize I2C communication
        self.ads = ADS.ADS1115(i2c) # Initialize the ADS1115 ADC
        self.ads.gain = 2/3
        self.chan = AnalogIn(self.ads, ADS.P0) # Initialize the analog input channel

        self.pH4_voltage = 4.39
        self.pH7_voltage = 3.93
        self.pH10_voltage = 3.56

    def voltage_to_pH(self, voltage): # Function to convert voltage to pH
        if voltage >= self.pH7_voltage:
            return 7.0 + (voltage - self.pH7_voltage) * (4.0 - 7.0) / (self.pH4_voltage - self.pH7_voltage)
        elif voltage < self.pH7_voltage:
            return 7.0 + (voltage - self.pH7_voltage) * (10.0 - 7.0) / (self.pH10_voltage - self.pH7_voltage)

    def read_sensor(self): # Function to read the sensor
        voltage = self.chan.voltage
        pH_value = self.voltage_to_pH(voltage)

        voltage_msg = Float32()
        voltage_msg.data = voltage
        self.publisher_voltage.publish(voltage_msg)

        pH_msg = Float32()
        pH_msg.data = pH_value
        self.publisher_ph.publish(pH_msg)

def main(args=None): # Main function
    rclpy.init(args=args)
    node = PHSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': # Run the main function
    main()
