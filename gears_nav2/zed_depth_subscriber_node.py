import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Bool

ZED_HEIGHT = 360
ZED_WIDTH = 640
MIN_DEPTH = 0.20
MAX_DEPTH = 10.0
ZED_DEPTH_LEFT_RIGHT_THRESH = 1.0  # 1.0 ORIG
ZED_FRONT_DEPTHTHRESH = 0.89  # 0.89 ORIG
action_dict = {0: "Left", 1: "Center", 2: "Right"}


class ZedDepthSubscriber(Node):
    def __init__(self):
        super().__init__("zed_depth_subscriber")
        self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.zed_depth_callback,
            10,
        )

        self.bridge = CvBridge()

        self.zed_is_near_wall_pub = self.create_publisher(
            Bool, "/zed/is_near_front_obs", 10
        )

        self.zed_is_obs_near_left_pub = self.create_publisher(
            Bool, "/zed/is_near_left_obs", 10
        )

        self.zed_is_obs_near_right_pub = self.create_publisher(
            Bool, "/zed/is_near_right_obs", 10
        )

        self.__is_near_wall = False
        self.__is_obs_near_left = False
        self.__is_obs_near_right = False

        timer_period = 1.0 / 20  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.wall_detect_timer_callback)

    def wall_detect_timer_callback(self):

        # self.is_near_wall_msg.data = self.__is_near_wall
        self.zed_is_near_wall_pub.publish(Bool(data=self.__is_near_wall))

        # self.is_obs_near_left_msg.data = self.__is_obs_near_left
        self.zed_is_obs_near_left_pub.publish(Bool(data=self.__is_obs_near_left))

        # self.is_obs_near_right_msg.data = self.__is_obs_near_right
        self.zed_is_obs_near_right_pub.publish(Bool(data=self.__is_obs_near_right))

    def zed_depth_callback(self, zed_msg):

        # height = zed_msg.height
        # width = zed_msg.width
        # depth_data = zed_msg.data

        # self.get_logger().info(f"Depth Data: {(len(depth_data))}")
        depth_image_cv_converted = self.bridge.imgmsg_to_cv2(
            zed_msg, desired_encoding="passthrough"
        )

        depth_image_float_converted = np.array(
            depth_image_cv_converted, dtype=np.float32
        )

        # self.get_logger().info(f"Height: {height} | Width: {width}")

        # str_kinect_range_image_c_ptr = self.__rangeFinder.getRangeImage(
        #     data_type="buffer"
        # )
        # # depthTemp = np.frombuffer(str_kinect_range_image, dtype=np.float32)

        depth_image_np = np.ctypeslib.as_array(
            depth_image_float_converted,
            (ZED_WIDTH * ZED_HEIGHT,),
        )
        # self.get_logger().info(f"Depth Data Len: {((depth_image_cv_converted.size))}")

        # contains_nan = np.isnan(depth_image_np).any()
        # contains_inf = np.isinf(depth_image_np).any()

        # # Logging the information
        # self.get_logger().info(f"Depth Data contains NaN: {contains_nan}")
        # self.get_logger().info(f"Depth Data contains Inf: {contains_inf}")

        # # Optionally, you can also log if all values are finite
        # all_finite = np.isfinite(depth_image_np).all()
        # self.get_logger().info(f"All depth data values are finite: {all_finite}")

        # maxRangeValueMask = np.isinf(depth_image_np)

        # depth_image_np[depth_image_np == -np.inf] = np.nan
        # depth_image_np[depth_image_np == +np.inf] = np.nan

        # Logging the information

        positive_inf_mask = np.isposinf(depth_image_np)

        # Create a mask where the value is negative infinity
        negative_inf_mask_or_nan_mask = np.isneginf(depth_image_np) | np.isnan(
            depth_image_np
        )

        # Replace positive infinity with 3.5
        depth_image_np[positive_inf_mask] = MAX_DEPTH

        # Replace negative infinity with 0.3
        depth_image_np[negative_inf_mask_or_nan_mask] = MIN_DEPTH

        max_depth = np.nanmax(depth_image_np)
        min_depth = np.nanmin(depth_image_np)
        # self.get_logger().info(f"Maximum depth value: {max_depth}")
        # self.get_logger().info(f"Minimum depth value: {min_depth}")

        # depth_image_np[maxRangeValueMask] = MAX_DEPTH
        normalized_depth = depth_image_np / MAX_DEPTH
        normalized_depth = normalized_depth.reshape((ZED_HEIGHT, ZED_WIDTH))

        # self.get_logger().info(f"Normalized Depth: {normalized_depth}")
        # self.get_logger().info(f"Depth Data: {(normalized_depth)}")
        # # normalized_depth = (normalized_depth*255).astype(np.uint8).reshape((self.__rangeFinder.getHeight(), self.__rangeFinder.getWidth()))
        # # normalized_depth = cv2.resize(normalized_depth,(32,19))

        # # If only 3 slices
        # 640
        one_third_slice_col = int(round(ZED_WIDTH / 3))  # 213.333333
        col_1 = one_third_slice_col - 1  # 213
        col_2 = ZED_WIDTH - one_third_slice_col - 1  # 427
        col_3 = ZED_WIDTH - 1  # 640
        # self.get_logger().info(f"Col 1: {col_1} | Col 2: {col_2} Col 3: {col_3}")

        one_third_slice_row = int(round(ZED_HEIGHT / 3))  # 120
        row_1 = one_third_slice_row - 1  # 120
        row_2 = ZED_HEIGHT - one_third_slice_row - 1  # 240
        row_3 = ZED_HEIGHT - 1  # 360
        # self.get_logger().info(f"Row 1: {row_1} | Row 2: {row_2} Row 3: {row_3}")

        slice_left = normalized_depth[row_1:row_3, 0:col_1]
        slice_mid = normalized_depth[row_1:row_3, col_1:col_2]
        slice_right = normalized_depth[row_1:row_3, col_2:col_3]

        total_slice_left = np.sum(slice_left)
        total_slice_mid = np.sum(slice_mid)
        total_slice_right = np.sum(slice_right)

        # depth_bins = [slice1, slice2, slice3]

        # print("depth_bins: ", depth_bins)
        # self.get_logger().info(f"Slice Left: {total_slice_left}")
        # self.get_logger().info(f"Slice Right: {total_slice_right}")

        # self.get_logger().info(
        #     f"Slice Left/Right: {total_slice_left/total_slice_right}"
        # )
        # self.get_logger().info(
        #     f"Slice Right/Left: {total_slice_right/total_slice_left}"
        # )

        total_depth_sum = (total_slice_left + total_slice_mid + total_slice_right) / (
            np.size(slice_left) + np.size(slice_mid) + np.size(slice_right)
        )
        total_depth_sum = 1 - total_depth_sum
        # self.get_logger().info(f"Depth % (Left+Mid+Right): {total_depth_sum}")

        if (total_slice_left / total_slice_right) > ZED_DEPTH_LEFT_RIGHT_THRESH:
            # self.__is_obs_near_left = False
            self.__is_obs_near_right = True
        else:
            self.__is_obs_near_right = False

        if (total_slice_right / total_slice_left) > ZED_DEPTH_LEFT_RIGHT_THRESH:
            self.__is_obs_near_left = True
        else:
            self.__is_obs_near_left = False
            # self.__is_obs_near_right = False

        if total_depth_sum >= ZED_FRONT_DEPTHTHRESH:
            self.__is_near_wall = True
        else:
            self.__is_near_wall = False
        # self.get_logger().info(f"Action: Going {action_dict[action]}")


def main(args=None):
    rclpy.init(args=args)

    zed_depth_subscriber = ZedDepthSubscriber()

    rclpy.spin(zed_depth_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    zed_depth_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
