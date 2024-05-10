import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from zed_interfaces.msg import ObjectsStamped
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

NEAR_DISTANCE_THRESH = 1.0  # < 1Meter
NEAR_DISTANCE_EXCLUDE_THRESH = 0.9
NEAR_DISTANCE_SIDE_THRESH = 1.7  # <1.5Meters
RATIO_Y_THRESH = 0.10


class ZedCustomNode(Node):
    def __init__(self):
        super().__init__("zed_2i_custom_node")
        # self._lidar_points = np.empty([])

        self._obj_det = ObjectsStamped()

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            ObjectsStamped,
            "/zed/zed_node/obj_det/objects",  # Replace '/laser_zed/zed_node/obj_det/objects_topic' with your actual laser zed/zed_node/obj_det/objects topic
            self.object_det_callback,
            qos_profile,  # Adjust the queue size as needed
        )

        self._obj_det_publisher = self.create_publisher(
            Bool, "/zed/is_object_detected_front", 10
        )

        self.zed_is_obs_near_left_pub = self.create_publisher(
            Bool, "/zed/is_object_detected_left", 10
        )

        self.zed_is_obs_near_right_pub = self.create_publisher(
            Bool, "/zed/is_object_detected_right", 10
        )

        self.__is_obj_detected = False
        self.__is_obj_detected_near_left = False
        self.__is_obj_detected_near_right = False

        timer_period = 1.0 / 20  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.zed_timer_callback)

    def object_det_callback(self, obj_det_msg):
        # lidar_points = np.array(lidar_msg.ranges)

        self._obj_det = obj_det_msg

        self.obj_detector()

    def zed_timer_callback(self):

        # self.is_object_detected_msg = Bool()
        # self.is_object_detected_msg.data = self.__is_obj_detected

        self._obj_det_publisher.publish(Bool(data=self.__is_obj_detected))
        self.zed_is_obs_near_left_pub.publish(
            Bool(data=self.__is_obj_detected_near_left)
        )
        self.zed_is_obs_near_right_pub.publish(
            Bool(data=self.__is_obj_detected_near_right)
        )

    def obj_detector(self):

        obj_det_count = len(self._obj_det.objects)

        nearest_object = None
        min_distance_x = float("inf")
        min_distance_y = float("inf")
        ratio_y = float("inf")

        for obj in self._obj_det.objects:

            distance_x = round(obj.position[0], 2)
            distance_y = round(obj.position[1], 2)
            # distance_z = round(obj.position[2], 2)

            if distance_x < min_distance_x:
                min_distance_x = distance_x
                min_distance_y = distance_y
                nearest_object = obj

        if nearest_object is not None:
            min_distance_x = round(
                min_distance_x, 3
            )  # Round the distance_x to three decimal places
            # if
            # self.get_logger().info(
            #     f"Nearest object is '{nearest_object.label}' with ID {nearest_object.label_id} at distance_x {min_distance_x}m | distance_y {min_distance_y}m"
            # )
            ratio_y = min_distance_y / min_distance_x

        self.get_logger().info(f"Dist X: {min_distance_x}")
        self.get_logger().info(f"Ratio Y: {ratio_y}")
        if (
            obj_det_count > 0
            and min_distance_x < NEAR_DISTANCE_THRESH
            and abs(ratio_y) < NEAR_DISTANCE_EXCLUDE_THRESH
            and nearest_object.label == "Person"  # Sport
        ):
            self.__is_obj_detected = True
        else:
            self.__is_obj_detected = False

        if (
            obj_det_count > 0
            and min_distance_x < NEAR_DISTANCE_SIDE_THRESH
            # and ratio_y > 0
            and ratio_y > RATIO_Y_THRESH
            and nearest_object.label == "Person"
        ):
            self.__is_obj_detected_near_left = True
        else:
            self.__is_obj_detected_near_left = False

        if (
            obj_det_count > 0
            and min_distance_x < NEAR_DISTANCE_SIDE_THRESH
            and ratio_y < -RATIO_Y_THRESH
            and nearest_object.label == "Person"
            # and ratio_y < 0
        ):
            self.__is_obj_detected_near_right = True
        else:
            self.__is_obj_detected_near_right = False
        # self.get_logger().info(f"Object Det Count: {obj_det_count}")


def main(args=None):
    rclpy.init(args=args)
    zed_2i_custom_node = ZedCustomNode()
    rclpy.spin(zed_2i_custom_node)
    zed_2i_custom_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
