import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float32, String

ZED_MIN_INCLINE_ANGLE_THRESHOLD = 3.0
ZED_MIN_DECLINE_ANGLE_THRESHOLD = -3.0


class ZedIMUSubscriber(Node):
    def __init__(self):
        super().__init__("zed_imu_subscriber")
        self.create_subscription(
            Imu,
            "/zed/zed_node/imu/data",
            self.zed_imu_callback,
            10,
        )
        self.zed_angle = 0.0
        self.zed_slope = "Flat"
        timer_period = 1.0 / 20  # seconds (20Hz)

        self.zed_angle_publisher = self.create_publisher(Float32, "/zed/angle", 10)
        self.zed_slope_publisher = self.create_publisher(String, "/zed/slope", 10)

        self.timer = self.create_timer(timer_period, self.zed_angle_pub_callback)

    def zed_angle_pub_callback(self):

        self.zed_angle_publisher.publish(Float32(data=self.zed_angle))

        self.zed_slope_publisher.publish(String(data=self.zed_slope))

    def zed_imu_callback(self, zed_msg):

        x = zed_msg.orientation.x
        y = zed_msg.orientation.y
        z = zed_msg.orientation.z
        w = zed_msg.orientation.w

        x_q, y_q, z_q = self.quaternion_to_euler(x, y, z, w)

        self.zed_angle = -(round(y_q, 2))

        if self.zed_angle >= ZED_MIN_INCLINE_ANGLE_THRESHOLD:
            self.zed_slope = "Incline"
        elif self.zed_angle <= ZED_MIN_DECLINE_ANGLE_THRESHOLD:
            self.zed_slope = "Decline"
        else:
            self.zed_slope = "Flat"

    def quaternion_to_euler(self, x, y, z, w):
        # Roll (phi)
        roll = np.arctan2(2 * (y * z + w * x), w**2 - x**2 - y**2 + z**2)

        # Pitch (theta)
        pitch = np.arcsin(2 * (w * y - x * z))

        # Yaw (psi)
        yaw = np.arctan2(2 * (x * y + w * z), w**2 + x**2 - y**2 - z**2)

        # Convert radians to degrees
        roll_degrees = np.degrees(roll)
        pitch_degrees = np.degrees(pitch)
        yaw_degrees = np.degrees(yaw)

        return roll_degrees, pitch_degrees, yaw_degrees


def main(args=None):
    rclpy.init(args=args)

    zed_imu_subscriber = ZedIMUSubscriber()

    rclpy.spin(zed_imu_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    zed_imu_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
