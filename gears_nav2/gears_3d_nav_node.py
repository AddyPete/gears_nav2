import rclpy
from geometry_msgs.msg import Twist
from controllers.gears_controller import RobotController
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from std_msgs.msg import Bool, String, Float32
from std_srvs.srv import Trigger
import time

STEER_MOTORS = 4
DRIVE_MOTORS = 4
ROBOT_WIDTH = 0.815971
ROBOT_HEIGHT = 0.7205

ROTATION_INCREMENT = 0.00872665
CORRECTION_ANGLE = 0.17  # 15 DEG
MAX_LINEAR = 0.3
MAX_ANGULAR = 0.3

MAX_LINEAR_OUTPUT = 3.0
MAX_ANGULAR_OUTPUT = 30
# TIME_STEP = 32


class GearsRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self._timestep = int(self.__robot.getBasicTimeStep())

        self.__supervisor = webots_node.robot.getSelf()
        self.trans_field = self.__supervisor.getField("translation")
        self.rot_field = self.__supervisor.getField("rotation")

        self.__inertial_unit = self.__robot.getDevice("inertial unit")
        self.__inertial_unit.enable(self._timestep)

        self.__rgb_camera = self.__robot.getDevice("Astra rgb")
        self.__depth_camera = self.__robot.getDevice("Astra depth")
        self.__rgb_camera.enable(self._timestep)
        self.__depth_camera.enable(self._timestep)
        self.__rgb_camera.recognitionEnable(self._timestep)

        wheel_motors = []
        steering_motors = []

        wheel_motor_names = ["FR_Wheel", "FL_Wheel", "BR_Wheel", "BL_Wheel"]
        steering_motor_names = ["FR_Control", "FL_Control", "BR_Control", "BL_Control"]

        for i in range(DRIVE_MOTORS):
            wheel_motors.append(self.__robot.getDevice(wheel_motor_names[i]))
            wheel_motors[i].setPosition(float("inf"))
            wheel_motors[i].setVelocity(0.0)

        for i in range(STEER_MOTORS):
            steering_motors.append(self.__robot.getDevice(steering_motor_names[i]))
            steering_motors[i].setPosition(0)

        self.__controller = RobotController(
            wheel_motors, steering_motors, ROBOT_WIDTH, ROBOT_HEIGHT
        )

        self.__target_twist = Twist()
        self.__is_obj_detected = Bool()
        self.__mode_msg = String()
        # self.__mode = "mirror"
        # self.arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=1)

        rclpy.init(args=None)
        self.__node = rclpy.create_node("my_robot_driver")
        self.__node.get_logger().info("Started Custom Driver Plugin")

        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)
        self.__node.create_subscription(String, "mode", self.__mode_callback, 1)

        self.broadcaster = TransformBroadcaster(self.__node)
        self.odom_publisher = self.__node.create_publisher(Odometry, "odom", 10)
        self.obj_det_publisher_ = self.__node.create_publisher(
            Bool, "/is_object_detected", 10
        )
        # self.mode_publisher = self.__node.create_publisher(String, "/mode", 10)

        self.reset_service = self.__node.create_service(
            Trigger, "reset_simulation", self.reset_simulation_callback
        )

        self.__mode = "mirrored"

        self.wall_left_collision_subscriber = self.__node.create_subscription(
            Bool, "/is_near_left_wall", self.wall_left_collision_callback, 10
        )

        self.wall_right_collision_subscriber = self.__node.create_subscription(
            Bool, "/is_near_right_wall", self.wall_right_collision_callback, 10
        )

        self._angle_requested_go_left = False
        self._angle_requested_go_right = False

        self.__node.create_subscription(
            Bool, "/angle_requested_go_left", self.angle_requested_go_left_callback, 10
        )
        self.__node.create_subscription(
            Bool,
            "/angle_requested_go_right",
            self.angle_requested_go_right_callback,
            10,
        )

        self.is_near_left_wall = False
        self.is_near_right_wall = False

        # self.angle_requested = False
        # self.__spin_mode_active = False

    def angle_requested_go_left_callback(self, angle_requested_left_msg):

        self._angle_requested_go_left = angle_requested_left_msg.data

        # self.__node.get_logger().info(
        #     f"SUB - Angle Req Go Left: {self._angle_requested_go_left}"
        # )

    def angle_requested_go_right_callback(self, angle_requested_right_msg):

        self._angle_requested_go_right = angle_requested_right_msg.data

        # self.__node.get_logger().info(
        #     f"SUB - Angle Req Go Right: {self._angle_requested_go_right}"
        # )

    def wall_left_collision_callback(self, msg):
        self.is_near_left_wall = msg.data

    def wall_right_collision_callback(self, msg):
        self.is_near_right_wall = msg.data

    def reset_simulation_callback(self, request, response):
        if self.__robot is None:
            self.get_logger().error(
                "Webots supervisor not initialized. Cannot reset simulation."
            )
            response.success = False
        # return response
        else:
            self.__node.get_logger().info("Resetting simulation...")
            self.__controller.reset_wheels()
            self.__target_twist.linear.x = 0.0
            self.__target_twist.angular.z = 0.0
            self.__controller.stop()
            self.__robot.simulationResetPhysics()
            # self.__robot.simulationReset()
            # reset_translation = [8.939, -0.001, 1.575]  # New position [x, y, z]
            # 8.938732223704125 -0.0009930059396084058 1.575153773185951
            # reset_rotation = [0.019, -0.100, 0.005, 0.181]

            # 0.128025 0 0.292574
            # 0 0 1 0

            reset_translation = [0.128, -0.001, 0.293]
            reset_rotation = [0, 0, 1, 0]

            # 0.018985947144367615 -0.9998069429081664 0.005060703869794486 0.18120325692260328
            self.trans_field.setSFVec3f(reset_translation)
            self.rot_field.setSFRotation(reset_rotation)
            self.__mode = "mirrored"
            self.__node.get_logger().info("Simulation reset successfully")
            response.success = True
        return response

    def __mode_callback(self, mode):
        self.__mode = mode.data

        if self.__mode == "spin":
            self.__controller.set_spin_mode()
        else:
            self.__controller.reset_wheels()

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        trans_values = self.trans_field.getSFVec3f()

        ori_values_quat = self.__inertial_unit.getQuaternion()
        ori_values_quat_rnd = np.array(ori_values_quat)

        linear_velocity = self.__target_twist.linear.x

        angular_velocity = self.__target_twist.angular.z

        if self.__mode == "mirrored":
            self.__controller.go_straight(linear_velocity)

            if angular_velocity > 0.0:  # LEFT STEER
                self.__controller.set_ackerman_steer(ROTATION_INCREMENT, self.__mode)
            elif angular_velocity < 0.0:  # RIGHT STEER
                self.__controller.set_ackerman_steer(-ROTATION_INCREMENT, self.__mode)

            else:
                self.__controller.reset_wheels()

            if self._angle_requested_go_left:
                # if self.is_near_right_wall:
                self.__controller.set_ackerman_angle(CORRECTION_ANGLE, self.__mode)
                # self.__node.get_logger().error("Go Left 15 DEG")
                # if self.is_near_left_wall:
                # self.__controller.set_ackerman_angle(-CORRECTION_ANGLE, self.__mode)
                # self.__node.get_logger().error("Left Wall 5 DEG")
                self._angle_requested_go_left = False
                self.__robot.step(256)
                # time.sleep(1)
            if self._angle_requested_go_right:
                self.__controller.set_ackerman_angle(-CORRECTION_ANGLE, self.__mode)
                # self.__node.get_logger().error("Go Right 15 DEG")
                self._angle_requested_go_right = False

                self.__robot.step(256)
        else:
            if linear_velocity != 0.0:
                self.__controller.go_spin(-linear_velocity)
            else:
                self.__controller.stop()

        x_pos, y_pos, z_pos = (trans_values[0], trans_values[1], trans_values[2])

        x_ori, y_ori, z_ori, w_ori = (
            ori_values_quat_rnd[0],
            ori_values_quat_rnd[1],
            ori_values_quat_rnd[2],
            ori_values_quat_rnd[3],
        )

        detected_objects = self.__rgb_camera.getRecognitionObjects()

        if detected_objects:
            self.__is_obj_detected.data = True
        else:
            self.__is_obj_detected.data = False

        self.__mode_msg.data = self.__mode

        self.obj_det_publisher_.publish(self.__is_obj_detected)
        # self.mode_publisher.publish(self.__mode_msg)

        self.publish_odometry(
            x_pos,
            y_pos,
            z_pos,
            x_ori,
            y_ori,
            z_ori,
            w_ori,
        )

    def publish_odometry(
        self,
        x_pos,
        y_pos,
        z_pos,
        x_ori,
        y_ori,
        z_ori,
        w_ori,
    ):
        odom_msg = Odometry()
        # odom_msg.header = Header()
        odom_msg.header.stamp = self.__node.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = x_pos  # Set X position to 1.0 for example
        odom_msg.pose.pose.position.y = y_pos  # Set Y position to 2.0 for example
        odom_msg.pose.pose.position.z = z_pos  # Set Z position to 0.0 for example
        odom_msg.pose.pose.orientation.x = x_ori
        odom_msg.pose.pose.orientation.y = y_ori
        odom_msg.pose.pose.orientation.z = z_ori
        odom_msg.pose.pose.orientation.w = w_ori

        # odom_msg.twist.twist.linear.x = forward_speed
        # odom_msg.twist.twist.angular.z = angular_speed

        self.odom_publisher.publish(odom_msg)

        t = TransformStamped()

        t.header.stamp = self.__node.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x_pos
        t.transform.translation.y = y_pos
        t.transform.translation.z = z_pos

        t.transform.rotation.x = x_ori
        t.transform.rotation.y = y_ori
        t.transform.rotation.z = z_ori
        t.transform.rotation.w = w_ori

        self.broadcaster.sendTransform(t)
