import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

LIDAR_RANGE_THRESHOLD = 1.5  # meters to turn
LIDAR_POINTS_THRESHOLD = 50  # points


LIDAR_RANGE_THRESHOLD_SIDE = 0.75
LIDAR_POINTS_THRESHOLD_SIDE = 20


class LaserWallDetection(Node):
    def __init__(self):
        super().__init__("laser_wall_detect_node")
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
        self._wall_left_collision_publisher = self.create_publisher(
            Bool, "/is_near_left_wall", 10
        )

        self._wall_right_collision_publisher = self.create_publisher(
            Bool, "/is_near_right_wall", 10
        )

        self.__is_near_wall = False
        self.__is_near_left_wall = False
        self.__is_near_right_wall = False
        timer_period = 1.0 / 20  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.wall_detect_timer_callback)

        self.is_near_wall_msg = Bool()
        self.is_near_wall_msg_left_msg = Bool()
        self.is_near_wall_msg_right_msg = Bool()

    def laser_callback(self, lidar_msg):
        lidar_points = np.array(lidar_msg.ranges)

        self._lidar_points = lidar_points

        self.wall_detector()
        self.side_wall_detector()

    def wall_detect_timer_callback(self):

        self.is_near_wall_msg.data = self.__is_near_wall
        self._wall_collision_publisher.publish(self.is_near_wall_msg)

        self.is_near_wall_msg_left_msg.data = self.__is_near_left_wall
        self._wall_left_collision_publisher.publish(self.is_near_wall_msg_left_msg)

        self.is_near_wall_msg_right_msg.data = self.__is_near_right_wall
        self._wall_right_collision_publisher.publish(self.is_near_wall_msg_right_msg)

    def wall_detector(self):

        try:
            lidar_points = self._lidar_points
            lidar_front_view = np.array(lidar_points[119:239])  # 60 EACH SIDE

            self._lidar_obstacle_count = np.sum(
                lidar_front_view <= LIDAR_RANGE_THRESHOLD
            )
            # self.get_logger().info(
            #     f"Lidar Obstacle Points: {self._lidar_obstacle_count}"
            # )
            # self.get_logger().info(f"Lidar  Points: {lidar_points.size}")
            if self._lidar_obstacle_count >= LIDAR_POINTS_THRESHOLD:

                self.__is_near_wall = True
            else:
                self.__is_near_wall = False
        except IndexError as e:
            self.get_logger().error(f"IndexError in need_to_rotate: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in need_to_rotate: {e}")

    def side_wall_detector(self):
        try:
            lidar_points = self._lidar_points

            lidar_left_side = np.array(lidar_points[89:134])
            lidar_right_side = np.array(lidar_points[224:269])

            self._lidar_obstacle_count_left = np.sum(
                lidar_left_side <= LIDAR_RANGE_THRESHOLD_SIDE
            )
            self._lidar_obstacle_count_right = np.sum(
                lidar_right_side <= LIDAR_RANGE_THRESHOLD_SIDE
            )

            self.get_logger().info(
                f"Lidar Obstacle Points Left: {self._lidar_obstacle_count_left}"
            )

            self.get_logger().info(
                f"Lidar Obstacle Points Right: {self._lidar_obstacle_count_right}"
            )

            if self._lidar_obstacle_count_left >= LIDAR_POINTS_THRESHOLD_SIDE:

                self.__is_near_left_wall = True
            else:
                self.__is_near_left_wall = False

            if self._lidar_obstacle_count_right >= LIDAR_POINTS_THRESHOLD_SIDE:

                self.__is_near_right_wall = True
            else:
                self.__is_near_right_wall = False

        except Exception as e:
            self.get_logger().error(f"Unexpected error in need_to_rotate: {e}")


def main(args=None):
    rclpy.init(args=args)
    laser_wall_detect_node = LaserWallDetection()
    rclpy.spin(laser_wall_detect_node)
    laser_wall_detect_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
