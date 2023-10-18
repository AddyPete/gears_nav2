import rclpy
from geometry_msgs.msg import Twist
from controllers.gears_controller import GearsController

STEER_MOTORS = 4
DRIVE_MOTORS = 4
ROBOT_WIDTH = 0.815971
ROBOT_HEIGHT = 0.7205


class GearsRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

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

        self.__controller = GearsController(
            wheel_motors, steering_motors, ROBOT_WIDTH, ROBOT_HEIGHT
        )

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node("my_robot_driver")
        self.__node.get_logger().info("Started Custom Driver Plugin")
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__controller.go_straight(5.0)

    #     self.__left_wheel_motor.setVelocity(self.__target_twist.linear.x)
    #     self.__right_wheel_motor.setVelocity(self.__target_twist.linear.x)
