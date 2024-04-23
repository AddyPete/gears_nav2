import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

LIDAR_RANGE_THRESHOLD = 1.4  # meters to turn
LIDAR_POINTS_THRESHOLD = 30  # points


class LaserLDWallDetection(Node):
    def __init__(self):
        super().__init__("laser_ld_wall_detect_node")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",  # Replace '/laser_scan_topic' with your actual laser scan topic
            self.laser_callback,
            10,  # Adjust the queue size as needed
        )
        self._lidar_points = np.empty([])
        self._wall_collision_publisher = self.create_publisher(
            Bool, "/is_near_wall", 10
        )

        self.__is_near_wall = False
        timer_period = 1.0 / 20  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.wall_detect_timer_callback)
        self.is_near_wall_msg = Bool()

    def laser_callback(self, lidar_msg):
        lidar_points = np.array(lidar_msg.ranges)

        self._lidar_points = lidar_points

        self.wall_detector()

    def wall_detect_timer_callback(self):

        self.is_near_wall_msg.data = self.__is_near_wall

        self._wall_collision_publisher.publish(self.is_near_wall_msg)

    def wall_detector(self):

        try:
            lidar_points = self._lidar_points
            lidar_front_view_left = np.array(
                lidar_points[0 : self.scale_lidar(30)]
            )  # +30 deg LEFT SIDES
            lidar_front_view_right = np.array(
                lidar_points[self.scale_lidar(330) : self.scale_lidar(360)]
            )  # + 30 deg RIGHT SIDE
            lidar_front_view = np.concatenate(
                (lidar_front_view_left, lidar_front_view_right)
            )

            self._lidar_obstacle_count = np.sum(
                lidar_front_view <= LIDAR_RANGE_THRESHOLD
            )
            self.get_logger().info(
                f"Lidar Obstacle Points: {self._lidar_obstacle_count}"
            )

            if self._lidar_obstacle_count >= LIDAR_POINTS_THRESHOLD:

                self.__is_near_wall = True
            else:
                self.__is_near_wall = False
        except IndexError as e:
            self.get_logger().error(f"IndexError in need_to_rotate: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in need_to_rotate: {e}")

    def scale_lidar(self, base_lidar_point):
        return int(round((base_lidar_point * 452) / 360, 0) - 1)


def main(args=None):
    rclpy.init(args=args)
    laser_ld_wall_detect_node = LaserLDWallDetection()
    rclpy.spin(laser_ld_wall_detect_node)
    laser_ld_wall_detect_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
