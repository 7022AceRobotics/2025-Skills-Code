# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       solkwak                                                      #
# 	Created:      4/9/2025, 9:33:39 AM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
import vex
import time
import math

def mps_to_rpm(mps):
    return mps / (VexSwerveModuleConstants.wheel_radius * math.pi * 2)

def rpm_to_mps(rpm):
    return rpm * (VexSwerveModuleConstants.wheel_radius * math.pi * 2)

def rpm_to_degps(rpm):
    return rpm * 360/60 # rpm/60 = rps, rps x 360 = deg/s

def degps_to_rpm(degps):
    return degps * 1/6

def make_rotations_universal(rotation):
    rotation = ((rotation + 180) % 360) - 180
    return rotation

# Used to convert rotational value from swerve drive
def make_rotations_universal2(rotation):
    rotation = ((rotation/2 + 180) % 360) - 180
    return rotation

def optimize_rotation(current_position, desired_position, clockwise):
    current_position = make_rotations_universal(current_position)
    desired_position = make_rotations_universal(desired_position)

    if not clockwise:
        if abs(current_position - desired_position) > 180:
            return 'forward'
        else:
            return 'backward'
    else:
        if abs(current_position - desired_position) > 180:
            return 'backward'
        else:
            return 'forward'
        
def multiply_matrices(m1, m2):
    # m1 is of size (n x k), m2 is of size (k x m)
    result = []
    for i in range(len(m1)):
        row = []
        for j in range(len(m2[0])):
            total = 0
            for k in range(len(m2)):
                total += m1[i][k] * m2[k][j]
            row.append(total)
        result.append(row)
    return result

def get_swerve_angle(x, y):
    hyp = (x**2 + y**2)**(1/2)
    if hyp > 0:
        x = x/ hyp
        y = y / hyp
    else:
        x = 1
        y = 0
    return radians_to_degrees(math.atan2(x, y))

# applies only to zero and one
def apply_deadband(inp, ran):
    if -ran <= inp <= ran:
        return 0
    if 1 - ran <= inp <= 1 + ran:
        return 1
    return inp

def inches_to_meters(inches):
    return inches / 39.37

def radians_to_degrees(rad):
    return rad / math.pi * 180

def check_if_angle_is_within_threshold(angle1, angle2, thresh):
    angle_diff = angle1 - angle2
    if abs(angle_diff) > thresh:
        return [True, angle_diff]
    else:
        return [False, angle_diff]
    

class VexSwerveModuleConstants():
    front_left_driving_port = 5
    front_left_turning_port = 3
    front_left_clockwise = True
    front_left_offset = 0

    front_right_driving_port = 13
    front_right_turning_port = 14
    front_right_clockwise = False
    front_right_offset = 0

    back_right_driving_port = 11
    back_right_turning_port = 12
    back_right_clockwise = False
    back_right_offset = 0

    back_left_driving_port = 2
    back_left_turning_port = 1
    back_left_clockwise = True
    back_left_offset = 0

    wheel_radius = inches_to_meters(3/2) # in meters
    turning_max_speed = 960 # deg/s
    driving_max_speed = 1 # m/s
    turning_gear_ratio = 2

class DrivingConstants():
    '''
    See Derivation of Inverse Kinematics for Swerve.pdf for a brilliant explanation of the inverse kinematics derivation
    https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
    '''

    wheel_base = inches_to_meters(7.5)
                        
    track_width = inches_to_meters(7)

    drive_kinematics = [
        (wheel_base/2, track_width/2),
        (wheel_base/2, -track_width/2),
        (-wheel_base/2, track_width/2),
        (-wheel_base/2, -track_width/2),
    ]

    inverse_kinematics = []
    for i in range(len(drive_kinematics)):
        inverse_kinematics.append(
            [i * 2, 0, 1, 0, -drive_kinematics[i][1]]
            )
        inverse_kinematics.append(
            [i * 2 + 1, 0, 0, 1, drive_kinematics[i][0]]
            )

class IntakeConstants():
    right_motor_port = 1
    left_motor_port = 1
    right_arm_motor_port = 1
    left_arm_motor_port = 1

class VexSwerveModule():
    def __init__(self, driving_port, turning_port, clockwise, offset, id):
        self.driving_motor = vex.Motor(driving_port)
        self.turning_motor = vex.Motor(turning_port)
        self.clockwise = clockwise
        self.offset = offset
        self.id = id
        #calculates how many degrees the encoder can be off before it stops the motor.
        self.threshold = 2 # degrees
        #calculates how many degrees the encoder can be off before it starts to turn the wheel.
        self.threshold2 = 0 # degrees
        self.motion_dict = {'forward':vex.FORWARD, 'backward':vex.REVERSE}


        self.turning_motor.set_position(0, vex.DEGREES)
        self.intended_position = 0

        #Slow down as x approaches a the desired angle
        # Graph generated from: https://www.desmos.com/calculator/sbtjfwfwmx. X represents angle currently at in degrees, y is speed in percent output
        #self.vel_compensation = lambda x: max(0, min(100, 22.74002 * x**(0.683772/2.03167) - 2.42683)) #<- Fast
        #self.vel_compensation = lambda x: max(0, min(100, 14.6363 * x**(-5644.21437/-13519.6038) - 1.2999)) #<- Slow
        self.vel_compensation = lambda x: max(0, min(100, 4.15943 * x**(1.2456/2.09046) - 2.5699)) #Super slow

        print(self.driving_motor.installed(), self.turning_motor.installed(), self.id)

    def get_state(self):
        return (rpm_to_mps(self.driving_motor.velocity()), make_rotations_universal(self.turning_motor.position()))
    
    def get_position(self):
        return (self.driving_motor.position(), make_rotations_universal(self.turning_motor.position()))
    
    def set_position(self, speed, rposition):

        #print(self.driving_motor.velocity(), self.id)

        # Adjust final position and speed to optimize movement.
        current_position = make_rotations_universal2(self.turning_motor.position())
        rposition, speed, rspeed = self.optimize(current_position, rposition, speed)
        diff = check_if_angle_is_within_threshold(current_position, rposition, self.threshold)
        rposition *= VexSwerveModuleConstants.turning_gear_ratio

        if self.clockwise:
            self.driving_motor.spin(vex.REVERSE, speed*100, vex.PERCENT)
        else:
            self.driving_motor.spin(vex.FORWARD, speed*100, vex.PERCENT)

        if abs(self.intended_position - rposition) > 10:
            if diff[0]:
                self.intended_position = rposition
                self.turning_motor.spin_to_position(rposition, vex.DEGREES, 80*rspeed, vex.PERCENT, False)
            else:
                self.turning_motor.stop()

    
    def reset_driving_encoder(self):
        self.driving_motor.reset_position()

    def reset_turning_encoder(self):
        self.turning_motor.reset_position()

    def get_driving_vel(self):
        return self.driving_motor.velocity()
    
    def get_turning_vel(self):
        return self.turning_motor.velocity()
    
    def optimize(self,current_angle, angle, speed):
        delta = angle - current_angle
        if abs(delta) > 90:
            speed *= -1
            angle = make_rotations_universal(angle + 180)
        
        if 0 >= angle - current_angle >= -90 or 90 <= angle - current_angle <= 180:
            rotation_speed = 1
        else:
            rotation_speed = -1
        return angle, speed, rotation_speed

class DriveSubsystem():
    def __init__(self, brain):
        self.brain = brain
        self.expander = vex.Triport(vex.Ports.PORT20)
        driving_port_dict = {1: vex.Ports.PORT1, 2: vex.Ports.PORT2, 3: vex.Ports.PORT3, 4: vex.Ports.PORT4, 5: vex.Ports.PORT5, 6: vex.Ports.PORT6, 7: vex.Ports.PORT7, 8: vex.Ports.PORT8, 9: vex.Ports.PORT9, 10: vex.Ports.PORT10, 11: vex.Ports.PORT11, 12: vex.Ports.PORT12, 13: vex.Ports.PORT13, 14: vex.Ports.PORT14}
        turning_port_dict = {1: vex.Ports.PORT1, 2: vex.Ports.PORT2, 3: vex.Ports.PORT3, 4: vex.Ports.PORT4, 5: vex.Ports.PORT5, 6: vex.Ports.PORT6, 7: vex.Ports.PORT7, 8: vex.Ports.PORT8, 9: vex.Ports.PORT9, 10: vex.Ports.PORT10, 11: vex.Ports.PORT11, 12: vex.Ports.PORT12, 13: vex.Ports.PORT13, 14: vex.Ports.PORT14}


        self.front_left_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.front_left_driving_port],
            turning_port_dict[VexSwerveModuleConstants.front_left_turning_port],
            VexSwerveModuleConstants.front_left_clockwise,
            VexSwerveModuleConstants.front_left_offset,
            1
        )
        self.front_right_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.front_right_driving_port],
            turning_port_dict[VexSwerveModuleConstants.front_right_turning_port],
            VexSwerveModuleConstants.front_right_clockwise,
            VexSwerveModuleConstants.front_right_offset,
            2
        )
        self.back_right_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.back_right_driving_port],
            turning_port_dict[VexSwerveModuleConstants.back_right_turning_port],
            VexSwerveModuleConstants.back_right_clockwise,
            VexSwerveModuleConstants.back_right_offset,
            3
        )
        self.back_left_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.back_left_driving_port],
            turning_port_dict[VexSwerveModuleConstants.back_left_turning_port],
            VexSwerveModuleConstants.back_left_clockwise,
            VexSwerveModuleConstants.back_right_offset,
            4
        )

    def chassisSpeedsToSwerveModuleStates(self, xspeed, yspeed, rotspeed):
        inverse_kinematics = DrivingConstants.inverse_kinematics
            
        module_final_states = multiply_matrices(inverse_kinematics, [[0],[0],[xspeed],[yspeed],[rotspeed]])
        module_desired_states = []

        for i in range(len(DrivingConstants.drive_kinematics)):
            x_final_speed = module_final_states[2*i][0]
            y_final_speed = module_final_states[2*i+1][0]
            # inefficient. get rid of it after implementing desaturate wheelSpeeds
            speed = min(VexSwerveModuleConstants.driving_max_speed, (x_final_speed**2 + y_final_speed**2)**(1/2))
            angle = make_rotations_universal(get_swerve_angle(x_final_speed, y_final_speed) - 90)
            module_desired_states.append((speed, angle))
        
        

        return module_desired_states
    
    def drive(self, xspeed, yspeed, rotspeed):
        module_states = self.chassisSpeedsToSwerveModuleStates(xspeed,yspeed,rotspeed)
        if rotspeed > 0.05:
            add_val = 90
            add_speed = -rotspeed/module_states[1][0]
            add_speed2 = rotspeed/module_states[0][0]
            add_speed3 = rotspeed/module_states[2][0]
            add_speed4 = rotspeed/module_states[3][0]
        elif rotspeed < -0.05:
            add_val = -90
            add_speed = -rotspeed/module_states[1][0]
            add_speed2 = rotspeed/module_states[0][0]
            add_speed3 = rotspeed/module_states[2][0]
            add_speed4 = rotspeed/module_states[3][0]
        else:
            add_val = 0
            add_speed = 1
            add_speed2 = 1
            add_speed3 = 1
            add_speed4 = 1
        self.front_left_module.set_position(module_states[0][0]*add_speed2, module_states[0][1] + add_val)
        self.front_right_module.set_position(module_states[1][0]*add_speed, module_states[1][1] + add_val)
        self.back_right_module.set_position(module_states[2][0]*add_speed3, module_states[2][1])
        self.back_left_module.set_position(module_states[3][0]*add_speed3, module_states[3][1])

class IntakeSubsystem():
    def __init__(self, brain):
        self.port_dict = {1:brain.three_wire_port.a, 2:brain.three_wire_port.b, 3:brain.three_wire_port.c, 4:brain.three_wire_port.d, 5:brain.three_wire_port.e, 6:brain.three_wire_port.f, 7:brain.three_wire_port.g, 8:brain.three_wire_port.h}
        self.right_motor = vex.Motor29(self.port_dict[IntakeConstants.right_motor_port])
        self.left_motor = vex.Motor29(self.port_dict[IntakeConstants.left_motor_port])

        self.right_intake_arm_motor = vex.Motor29(self.port_dict[IntakeConstants.right_arm_motor_port])
        self.left_intake_arm_motor = vex.Motor29(self.port_dict[IntakeConstants.left_arm_motor_port])

    def move_intake_arms(self, speed):
        self.right_intake_arm_motor.spin(vex.FORWARD, speed, vex.PERCENT)
        self.left_intake_arm_motor.spin(vex.REVERSE, speed, vex.PERCENT)
    
    def intake(self, speed):
        self.right_motor.spin(vex.FORWARD, speed, vex.PERCENT)
        self.left_motor.spin(vex.REVERSE, speed, vex.PERCENT)

    def full_intake_motion(self):
        self.intake(1)
        self.move_intake_arms(-0.5)
        vex.wait(1000)
        self.move_intake_arms(0.5)
        vex.wait(1000)
        self.stop_intake()
        self.move_intake_arms(0)

    def stop_intake(self):
        self.right_motor.stop()
        self.left_motor.stop()

class ArmSubsystem():
    def __init__(self, arm_motor_port):
        self.arm_motor = vex.Motor29(arm_motor_port)
    def deploy(self):
        self.arm_motor.spin(vex.FORWARD, 80, vex.PERCENT)
    def stop(self):
        self.arm_motor.stop()

class Button():
    def __init__(self, toggle, controller, controller_button):
        self.controller_button_dict = {
            "A":controller.buttonA, 
            "B":controller.buttonB, 
            "X":controller.buttonX, 
            "Y":controller.buttonY, 
            "Up":controller.buttonUp, 
            "Down":controller.buttonDown, 
            "Left":controller.buttonLeft, 
            "Right":controller.buttonRight, 
            "L1":controller.buttonL1, 
            "L2":controller.buttonL2, 
            "R1":controller.buttonR1, 
            "R2":controller.buttonR2
            }
        self.controller_button = self.controller_button_dict[controller_button]

        # CLASS VARS FOR toggle FUNC
        self.threads_toggle = None
        self.toggle_prev_pressed = False


    def is_getting_pressed(self, func, arg_tup, func2, arg_tup2):
        if self.controller_button.pressing():
            func(*arg_tup)
        else:
            func2(*arg_tup2)

    def toggle(self, func, arg_tup, func2, arg_tup2):
        if self.controller_button.pressing():
            if not self.toggle_prev_pressed:
                self.toggle_prev_pressed = True
                if self.threads_toggle == None:
                    self.threads_toggle = vex.Thread(func, arg_tup)
                else:
                    self.threads_toggle.stop()
                    func2(*arg_tup2)
                    self.threads_toggle = None
        else:
            self.toggle_prev_pressed = False


# Brain should be defined by default
brain=vex.Brain()
drive_subsystem = DriveSubsystem(brain)
intake_subsystem = IntakeSubsystem(brain)
controller = vex.Controller()

if __name__ == "__main__":
    motor1 = vex.Motor(2)
    while True:
        drive_subsystem.drive(
            controller.axis3.position()/100,
            controller.axis4.position()/100,
            controller.axis1.position()/100
        # apply_deadband(controller.axis3.position(), 0.05),
        # apply_deadband(controller.axis4.position(), 0.05),
        # apply_deadband(controller.axis1.position(), 0.05),
        )
        vex.wait(10, vex.MSEC)

        
