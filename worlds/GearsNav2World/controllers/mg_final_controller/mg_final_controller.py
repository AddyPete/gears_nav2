import math

class RobotController:

    
    def __init__(self,wheel_motors,steering_motors,a,b):
        
        # WHEEL and STEERING MOTORS ID
        # 0 - Front Right
        # 1 - Front Left
        # 2 - Back Right
        # 3 - Back Left
        
        self.wheel_motors = []
        self.steering_motors = []
        self.velocities = [0,0,0,0]
        self.gen_velocity = 0
        self.angle_rad = 0.785 
        self.deg = 0
        self.a = a
        self.b = b
        self.ackerman_rot_h = 0
        self.ackerman_rot_l = 0
        self.mode = "default"


        for i in range (4):
            self.wheel_motors.append(wheel_motors[i])
            self.steering_motors.append(steering_motors[i]) 
        
        self.a = a
        self.b = b


    def go_straight(self,velocity):
        
        # print ("STRAIGHT HIGH {0}".format(math.degrees(self.ackerman_rot_h)))
        # print ("LSTRAIGHT LOW  {0}".format(math.degrees(self.ackerman_rot_l)))

        # print ("HIGH {0}".format(math.degrees(self.ackerman_rot_h)))
        # print ("LOW  {0}".format(math.degrees(self.ackerman_rot_l)))

        
        velocities = [velocity,velocity,velocity,velocity]
        
        if self.mode == "ackerman":

            # RIGHT SIDE
            self.velocities[0] = velocity
            self.velocities[2] = self.velocities[0] * (

                            math.sin(abs((self.ackerman_rot_l))) *       
                            (math.tan(abs((self.ackerman_rot_h))) + 1)
                            / math.tan(abs((self.ackerman_rot_h)))

            )

            # LEFT SIDE
            self.velocities[1] = self.velocities[0] * (

                            math.sin(abs(self.ackerman_rot_l)) /
                            math.sin(abs(self.ackerman_rot_h))

            )
            self.velocities[3] = self.velocities[0] * (


                            math.sin(abs(self.ackerman_rot_l)) /
                            math.tan(abs(self.ackerman_rot_h))

            )
        elif self.mode == "mirrored":
            self.velocities[0] = velocity
            self.velocities[2] = self.velocities[0] 

            # LEFT SIDE
            self.velocities[1] = self.velocities[0] * (

                            math.sin(abs(self.ackerman_rot_l)) /
                            math.sin(abs(self.ackerman_rot_h))

            )
            self.velocities[3] = self.velocities[0]
        #self.velocities [1] = math.tan(abs(math.degrees(self.ackerman_rot_h)))

        #math.tan(abs(math.degrees(self.ackerman_rot_h)))
        
        #math.tan(abs(math.degrees(self.ackerman_rot_h)))+1
        #math.sin(abs(math.degrees(self.ackerman_rot_l)))

        #print ("VEL {0}".format(self.velocities))

        self.set_wheel_speed(velocities)

        # print ("HIGH {0}".format(math.degrees(self.ackerman_rot_h)))
        # print ("LOW  {0}".format(math.degrees(self.ackerman_rot_l)))

    def go_backwards(self,velocity):
    
        velocities = [-velocity,-velocity,-velocity,-velocity]
        self.set_wheel_speed(velocities)
  
    def stop(self):

        velocities = [0,0,0,0]
        self.set_wheel_speed(velocities)

    def reset_wheels(self):
        
        self.ackerman_rot_h = 0
        self.ackerman_rot_l = 0
        self.deg = 0
        self.velocities = [0,0,0,0]
        steers = [0,0,0,0]
        self.__set_steer(steers)
        # self.set_wheel_speed(self.velocities)

    def set_spin_mode(self):

        steering_motors_theta = [self.angle_rad,-self.angle_rad,-self.angle_rad,self.angle_rad]
        self.__set_steer(steering_motors_theta)

    def go_spin(self,velocity,direction):
                
        if direction == "counter_clockwise":
        
            velocities_counter_cw = [velocity,-velocity,velocity,-velocity]
            self.set_wheel_speed(velocities_counter_cw)
                
        elif direction == "clockwise":
        
            velocities_cw = [-velocity,velocity,-velocity,velocity]
            self.set_wheel_speed(velocities_cw)
    
    def set_crab_mode(self,rot_inc,direction):
        

        
        tmp = self.deg

        if direction == "right" and self.deg > -self.angle_rad:
            tmp += -rot_inc
        elif direction == "left" and self.deg < self.angle_rad:
            tmp += rot_inc


        if tmp > self.angle_rad or tmp < -self.angle_rad:
            return
        self.deg = tmp
 
        steering_motors_theta = [self.deg,self.deg,self.deg,self.deg]
        self.__set_steer(steering_motors_theta)


    
    def set_ackerman_steer(self, rot_inc, mode):

        # print ("HIGH BEF {0}".format(math.degrees(self.ackerman_rot_h)))
        # print ("LOW BEF {0}".format(math.degrees(self.ackerman_rot_l)))



        self.mode = mode
        #print ("ROT INCREMENT: {0}".format(rot_inc))
        curr_deg = self.deg
        self.deg += math.degrees(rot_inc)
        
        
        if (abs(self.deg) >= 45):
            self.deg = curr_deg
            return
        
        self.ackerman_rot_h += (rot_inc)


        if self.ackerman_rot_h == 0:
            return

        print ("HIGH AFT {0}".format(math.degrees(self.ackerman_rot_h)))
        print ("LOW AFT {0}".format(math.degrees(self.ackerman_rot_l)))

        print ("CHECK DENOM: "+str((2 * a + b / math.tan(abs((self.ackerman_rot_h))))))


        if self.mode == "ackerman":
            phi = math.atan( math.tan(abs((self.ackerman_rot_h))) / (math.tan(abs((self.ackerman_rot_h)))+1 ) )
        # print ("PHI {0}".format(phi))
        else:
            phi = math.atan( self.b / (2 * self.a + self.b / math.tan(abs((self.ackerman_rot_h)))) )
            # print ("STOP")
        
        print ("A: {0} B: {1}".format(a,b))

        if self.ackerman_rot_h > 0:
            self.ackerman_rot_l = phi
        else:
            self.ackerman_rot_l = -phi

        # print ("HIGH {0}".format(math.degrees(self.ackerman_rot_h)))
        # print ("LOW  {0}".format(math.degrees(self.ackerman_rot_l)))
        
        
        # print (rot_inc)
        # 0 = R 1 = L

        if mode == "ackerman":
            if (self.ackerman_rot_h > 0): #### LEFT STEER
                steers = [self.ackerman_rot_l,self.ackerman_rot_h,0,0]
            else: #### RIGHT STEER
                steers = [self.ackerman_rot_h,self.ackerman_rot_l,0,0]
        else:
            if (self.ackerman_rot_h > 0): #### LEFT STEER
                steers = [self.ackerman_rot_l,self.ackerman_rot_h,-self.ackerman_rot_l,-self.ackerman_rot_h]
            else: #### RIGHT STEER
                steers = [self.ackerman_rot_h,self.ackerman_rot_l,-self.ackerman_rot_h,-self.ackerman_rot_l]            
        
        self.__set_steer(steers)

    def set_wheel_speed(self,velocities):
        for i in range (4):
            self.wheel_motors[i].setVelocity(velocities[i])      

    def __set_steer(self,steers):
        for i in range (4):
            self.steering_motors[i].setPosition(steers[i])  

        
                    
       
"""mine_gears_rover controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

#from controller import Robot, Camera, RangeFinder

import os
import random
# if os.getenv('ROS_DISTRO') is not None:
    # from webots_ros2_driver_webots.controller import Robot, Camera, RangeFinder
# else:
from controller import Robot, Camera, RangeFinder, Lidar


#from controller import Robot, Camera, RangeFinder

robot = Robot()
lidar = Lidar('LDS-01')
# camera = Camera('kinect color')
# rangefinder = RangeFinder ('kinect range')

timestep = int(robot.getBasicTimeStep())

wheel_motors = []
steering_motors = []

wheel_motor_names = ['FR_Wheel','FL_Wheel','BR_Wheel','BL_Wheel']
steering_motor_names = ['FR_Control','FL_Control','BR_Control','BL_Control']

robot.keyboard.enable(timestep)
robot.keyboard = robot.getKeyboard()

lidar.enable(timestep)
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
# rangefinder.enable(timestep)

for i in range(4):
    wheel_motors.append(robot.getDevice(wheel_motor_names[i]))
    wheel_motors[i].setPosition(float('inf'))
    wheel_motors[i].setVelocity(0.0)

for i in range(4):
    steering_motors.append(robot.getDevice(steering_motor_names[i]))
    steering_motors[i].setPosition(0)

max_velocity = 4.0
reset = 0
rotation = 0
rot_inc = 0.00872665 #0.5 DEG
# rot_inc = 0.05
curr_velocity = max_velocity
a = 0.815971
b = 0.7205

controller = RobotController(wheel_motors,steering_motors,a,b)

delta_time = 0
last_time = robot.getTime()
mode = "mirror"

action = 0


print ("-------------------RESET------------------------")


while robot.step(timestep) != -1:
    

    ###########################
    # action = random.randint(0, 2)
    
    # print("Action: ============= {0}".format(action))

    # controller.go_straight(curr_velocity)

    # if action == 0: # INC LEFT
        # controller.set_ackerman_steer(rot_inc,mode)
    # elif action == 2: # INC RIGHT
        # controller.set_ackerman_steer(-rot_inc, mode)
    # elif action == 1:
        # pass

    #################################


    key = robot.keyboard.getKey()
    
    controller.go_straight(curr_velocity)

    if key != -1:
        pass
        #print (key)
    
    if key == 87: # W KEY
        controller.go_straight(curr_velocity)
    
    elif key == 49: #1
        mode = "mirror"
    elif key == 50: # 2 
        mode = "ackerman"

    elif key == 68: #D KEY
        controller.set_ackerman_steer(-rot_inc,mode)

    elif key == 65: # A KEY
        controller.set_ackerman_steer(rot_inc,mode)
        
    elif key == 83: # W KEY
        controller.go_backwards(curr_velocity)

    elif key == 54: # 6 KEY 
        controller.set_crab_mode(rot_inc,"left")

    elif key == 55: # 7 KEY
        controller.set_crab_mode(rot_inc,"right")

    elif key == 56: # 8 KEY
        controller.set_spin_mode()

    elif key == 57: # ( KEY
        controller.go_spin(curr_velocity,"counter_clockwise")

    elif key == 48: # ) KEY   
        controller.go_spin(curr_velocity,"clockwise")
                  
    elif key == 32: # SPACEBAR
        controller.stop()
    
    elif key == 61: # + KEY
        curr_velocity += 0.1
        controller.set_wheel_speed([curr_velocity,curr_velocity,curr_velocity,curr_velocity])
    elif key == 45: # - KEY
        curr_velocity -= 0.1
        controller.set_wheel_speed([curr_velocity,curr_velocity,curr_velocity,curr_velocity])
    elif key == 88: # X KEY 
        controller.reset_wheels()
        
        
        
        