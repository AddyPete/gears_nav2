import rclpy
from geometry_msgs.msg import Twist
from controllers.gears_controller import RobotController
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math

STEER_MOTORS = 4
DRIVE_MOTORS = 4
ROBOT_WIDTH = 0.815971
ROBOT_HEIGHT = 0.7205

ROTATION_INCREMENT = 0.00872665

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
        # self.__mode = "mirror"
        # self.arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=1)

        rclpy.init(args=None)
        self.__node = rclpy.create_node("my_robot_driver")
        self.__node.get_logger().info("Started Custom Driver Plugin")

        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)

        self.broadcaster = TransformBroadcaster(self.__node)
        self.odom_publisher = self.__node.create_publisher(Odometry, "odom", 10)

        self.__mode = "mirrored"
        self.__spin_mode_active = False

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        trans_values = self.trans_field.getSFVec3f()

        ori_values_quat = self.__inertial_unit.getQuaternion()
        ori_values_quat_rnd = np.array(ori_values_quat)

        linear_velocity = self.__target_twist.linear.x

        angular_velocity = self.__target_twist.angular.z

        self.__controller.go_straight(linear_velocity)

        if angular_velocity > 0.0:  # LEFT STEER
            self.__controller.set_ackerman_steer(ROTATION_INCREMENT, self.__mode)
        elif angular_velocity < 0.0:  # RIGHT STEER
            self.__controller.set_ackerman_steer(-ROTATION_INCREMENT, self.__mode)

        else:
            self.__controller.reset_wheels()
        # if linear_speed  != 0.0 or angular_speed != 0.0:
        #     self.__node.get_logger().info(f"Command Received")

        #     if linear_speed == 0.0:
        #         self.__mode = "spin"
        #     else:
        #         self.__mode = "mirror"

        #     if self.__mode == "spin":

        #         # self.__mode = "spin"

        #         if not self.__spin_mode_active:
        #             self.__controller.set_spin_mode()
        #             # time.sleep(1)
        #             self.__spin_mode_active = True

        #         final_angular_speed = abs(
        #             np.interp(
        #                 angular_speed,
        #                 (
        #                     math.radians(-MAX_ANGULAR),
        #                     math.radians(MAX_ANGULAR),
        #                 ),
        #                 (-MAX_LINEAR_OUTPUT, MAX_LINEAR_OUTPUT),
        #             )
        #         )
        #         if angular_speed > 0:
        #             self.__controller.go_spin(final_angular_speed, "counter_clockwise")
        #         else:
        #             self.__controller.go_spin(final_angular_speed, "clockwise")

        #     else:
        #         # self.__mode = "mirror"
        #         # linear_speed = abs(angular_speed)
        #         self.__controller.go_straight(linear_speed)
        #         self.__controller.set_ackerman_steer(angular_speed)
        #         self.__spin_mode_active = False

        # else:
        #     self.__node.get_logger().info(f"No Command Received")
        #     self.__controller.reset_wheels()
        #     self.__controller.stop()
        #     self.__spin_mode_active = False

        # if linear_speed != 0.0 and angular_speed == 0.0:
        #     self.__node.get_logger().info(f"Reset Wheels")
        #     self.__controller.reset_wheels()

        # self.__node.get_logger().info(
        #     f" Linear: {linear_speed} Angular: {angular_speed}"
        # )

        x_pos, y_pos, z_pos = (trans_values[0], trans_values[1], trans_values[2])

        x_ori, y_ori, z_ori, w_ori = (
            ori_values_quat_rnd[0],
            ori_values_quat_rnd[1],
            ori_values_quat_rnd[2],
            ori_values_quat_rnd[3],
        )

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
        # self.__node.get_logger().info("Odometry message published")
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
