#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3
import board
import busio
import math
import time
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
from scipy.spatial.transform import Rotation

class IMUSensor(Node):
    def __init__(self):
        super().__init__("imu_sensor")

        qos_profile = QoSProfile(depth=10)

        # Create publishers for each sensor on the IMU
        self.orientation_pub = self.create_publisher(Vector3, 'orientation_sensor', qos_profile)

        self.log = self.get_logger()
        self.connected = False
        self.sensor_init()
        if self.connected: self.create_timer(0.003, self.pub_sensors)

    def sensor_init(self):

        # Try to connect to sensor.
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
            self.sensor = BNO08X_I2C(self.i2c)
            time.sleep(0.5)
            self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        except Exception as e:
            self.log.info(str(e))
            self.log.warn("Cannot connect to BNO08X. Ignore this if IMU is unplugged")
            exit()
        else:
            self.connected = True

    def pub_sensors(self):
        # When receiving the quaternino from the sensor, a KeyError is sometimes generated due to a bad packet so a try-except block is used to avoid that error.
        try:
            #           self.log.info(str(self.sensor.calibration_status))
            # Receive the sensor's quaternion output
            orientation = self.sensor.quaternion
            # Check that said quaternion exists
            if orientation == tuple((0, 0, 0, 0)):
                raise Exception("Quaternion Does Not Exist")
            # Convert those quaternions to euler angle between -180 and 180
            roll, pitch, yaw = Rotation.from_quat(orientation).as_euler('xyz', degrees=True)
            # Create ros msg and convert the euler angles to the standard format of 0 to 360 degrees
            msg = Vector3()
            msg.x = (roll + 180) % 360
            msg.y = (pitch +180) % 360
            msg.z = (yaw + 180) % 360
            self.orientation_pub.publish(msg)
        except Exception as e:
            #self.log.info("Invalid Packet: " + str(e))
            return


def main(args=None):
    rclpy.init(args=args)
    imu_sensor = IMUSensor()
    rclpy.spin(imu_sensor)
    imu_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
