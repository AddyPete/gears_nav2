import math


class RobotController:
    def __init__(self, wheel_motors, steering_motors, a, b):
        # WHEEL and STEERING MOTORS ID
        # 0 - Front Right
        # 1 - Front Left
        # 2 - Back Right
        # 3 - Back Left

        self.wheel_motors = []
        self.steering_motors = []
        self.velocities = [0, 0, 0, 0]
        self.gen_velocity = 0
        self.angle_rad = 0.785
        self.deg = 0
        self.a = a
        self.b = b
        self.ackerman_rot_h = 0
        self.ackerman_rot_l = 0

        for i in range(4):
            self.wheel_motors.append(wheel_motors[i])
            self.steering_motors.append(steering_motors[i])

        self.a = a
        self.b = b

    def go_straight(self, velocity):

        velocities = [velocity, velocity, velocity, velocity]

        self.velocities[0] = velocity
        self.velocities[2] = self.velocities[0]

        # LEFT SIDE
        self.velocities[1] = self.velocities[0]
        self.velocities[3] = self.velocities[0]

        self.set_wheel_speed(velocities)

    def go_backwards(self, velocity):
        velocities = [-velocity, -velocity, -velocity, -velocity]
        self.set_wheel_speed(velocities)

    def stop(self):
        velocities = [0, 0, 0, 0]
        self.set_wheel_speed(velocities)

    def reset_wheels(self):
        self.ackerman_rot_h = 0
        self.ackerman_rot_l = 0
        self.deg = 0
        self.velocities = [0, 0, 0, 0]
        steers = [0, 0, 0, 0]
        self.__set_steer(steers)

    def set_spin_mode(self):
        steering_motors_theta = [
            self.angle_rad,
            -self.angle_rad,
            -self.angle_rad,
            self.angle_rad,
        ]
        self.__set_steer(steering_motors_theta)

    def go_spin(self, velocity):

        velocities_cw = [-velocity, velocity, -velocity, velocity]
        self.set_wheel_speed(velocities_cw)

    def set_crab_mode(self, rot_inc, direction):
        tmp = self.deg

        if direction == "right" and self.deg > -self.angle_rad:
            tmp += -rot_inc
        elif direction == "left" and self.deg < self.angle_rad:
            tmp += rot_inc

        if tmp > self.angle_rad or tmp < -self.angle_rad:
            return
        self.deg = tmp

        steering_motors_theta = [self.deg, self.deg, self.deg, self.deg]
        self.__set_steer(steering_motors_theta)

    def set_ackerman_steer(self, rot_inc, mode):

        curr_deg = self.deg
        self.deg += math.degrees(rot_inc)

        if abs(self.deg) >= 45:
            self.deg = curr_deg
            return

        self.ackerman_rot_h += rot_inc

        if self.ackerman_rot_h == 0:
            return

        phi = math.atan(
            self.b / (2 * self.a + self.b / math.tan(abs((self.ackerman_rot_h))))
        )

        if self.ackerman_rot_h > 0:
            self.ackerman_rot_l = phi
        else:
            self.ackerman_rot_l = -phi

        if self.ackerman_rot_h > 0:  #### LEFT STEER
            steers = [
                self.ackerman_rot_l,
                self.ackerman_rot_h,
                -self.ackerman_rot_l,
                -self.ackerman_rot_h,
            ]
        else:  #### RIGHT STEER
            steers = [
                self.ackerman_rot_h,
                self.ackerman_rot_l,
                -self.ackerman_rot_h,
                -self.ackerman_rot_l,
            ]

        self.__set_steer(steers)

    def set_wheel_speed(self, velocities):
        for i in range(4):
            self.wheel_motors[i].setVelocity(velocities[i])

    def __set_steer(self, steers):
        for i in range(4):
            self.steering_motors[i].setPosition(steers[i])
