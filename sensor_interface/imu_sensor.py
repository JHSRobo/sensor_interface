#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3
import board
import busio
import adafruit_bno055

class IMUSensor(Node):
    def __init__(self):
        super().__init__("imu_sensor")

        qos_profile = QoSProfile(depth=10)

        # Create publishers for each sensor on the IMU
        self.accelerometer_pub = self.create_publisher(Vector3, 'accelerometer', qos_profile)
        self.magnetometer_pub = self.create_publisher(Vector3, 'magnetometer', qos_profile)
        self.gyro_pub = self.create_publisher(Vector3, 'gyroscope', qos_profile)
        self.orientation_pub = self.create_publisher(Vector3, 'orientation_sensor', qos_profile)
        self.linear_accelerometer_pub = self.create_publisher(Vector3, 'linear_accelerometer', qos_profile)
        self.gravitometer_pub = self.create_publisher(Vector3, 'gravitometer', qos_profile)

        self.logger = self.get_logger()
        self.connected = False
        self.sensor_init()
        if self.connected: self.create_timer(0.003, self.pub_sensors)

    def sensor_init(self):

        # Try to connect to sensor.
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        except:
            self.logger.warn("Cannot connect to BNO055. Ignore this if IMU is unplugged")
            exit()
        else:
            self.connected = True
    
    def pub_sensors(self):

        # Publish Acceleration
        try: acceleration_msg = self.create_vector_msg(self.sensor.acceleration)
        except: pass
        else: 
            if acceleration_msg is not None: self.accelerometer_pub.publish(acceleration_msg)

        # Publish Magnetometry
        try: magnetometry_msg = self.create_vector_msg(self.sensor.magnetic)
        except: pass
        else: 
            if magnetometry_msg is not None: self.magnetometer_pub.publish(magnetometry_msg)

        # Publish Magnetometry
        try: gyro_msg = self.create_vector_msg(self.sensor.gyro)
        except: pass
        else: 
            if gyro_msg is not None: self.gyro_pub.publish(gyro_msg)

        # Publish Euler Orientation
        try: orientation_msg = self.create_vector_msg(self.sensor.euler)
        except: pass
        else: 
            if orientation_msg is not None: self.orientation_pub.publish(orientation_msg)

        # Publish Linear Acceleration (Without Gravity)
        try: linear_acceleration_msg = self.create_vector_msg(self.sensor.linear_acceleration)
        except: pass
        else: 
            if linear_acceleration_msg is not None: self.linear_accelerometer_pub.publish(linear_acceleration_msg)

        # Publish Gravitometry (Without Linear Acceleration)
        try: gravitometry_msg = self.create_vector_msg(self.sensor.gravity)
        except: pass
        else: 
            if gravitometry_msg is not None: self.gravitometer_pub.publish(gravitometry_msg)

    def create_vector_msg(self, measurement): 
        vector_msg = Vector3()
        if None in measurement:
            return None
        vector_msg.x = round(float(measurement[0]), 2)
        vector_msg.y = round(float(measurement[1]), 2)
        vector_msg.z = round(float(measurement[2]), 2)

        return vector_msg


def main(args=None):
    rclpy.init(args=args)
    imu_sensor = IMUSensor()
    rclpy.spin(imu_sensor)
    imu_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
