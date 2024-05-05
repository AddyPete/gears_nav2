import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from zed_interfaces.msg import ObjectsStamped
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

NEAR_DISTANCE_THRESH = 1.0  # < 1Meter


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
            Bool, "/zed/is_object_detected", 10
        )

        self.__is_obj_detected = False
        timer_period = 1.0 / 20  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.zed_timer_callback)

    def object_det_callback(self, obj_det_msg):
        # lidar_points = np.array(lidar_msg.ranges)

        self._obj_det = obj_det_msg

        self.obj_detector()

    def zed_timer_callback(self):

        self.is_object_detected_msg = Bool()
        self.is_object_detected_msg.data = self.__is_obj_detected

        self._obj_det_publisher.publish(self.is_object_detected_msg)

    def obj_detector(self):

        obj_det_count = len(self._obj_det.objects)

        nearest_object = None
        min_distance = float("inf")

        for obj in self._obj_det.objects:

            distance = obj.position[0]
            if distance < min_distance:
                min_distance = distance
                nearest_object = obj

        if nearest_object is not None:
            min_distance = round(
                min_distance, 3
            )  # Round the distance to three decimal places
            # self.get_logger().info(
            #     f"Nearest object is '{nearest_object.label}' with ID {nearest_object.label_id} at distance {min_distance} meters."
            # )

        if (
            obj_det_count > 0
            and min_distance < NEAR_DISTANCE_THRESH
            and nearest_object.label == "Person"
        ):
            self.__is_obj_detected = True
        else:
            self.__is_obj_detected = False

        # self.get_logger().info(f"Object Det Count: {obj_det_count}")


def main(args=None):
    rclpy.init(args=args)
    zed_2i_custom_node = ZedCustomNode()
    rclpy.spin(zed_2i_custom_node)
    zed_2i_custom_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
