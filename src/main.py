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
    rotation = rotation % 360
    if rotation < 0:
        rotation = 360 + rotation
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
    angle_diff = (angle1 - angle2 + 540) % 360 - 180
    if abs(angle_diff) > thresh:
        return [True, angle_diff]
    else:
        return [False, angle_diff]

def testing(encoder, expander, motor, required_position):
    encoder = vex.Encoder(brain.three_wire_port.a)
    expander = vex.Triport(vex.Ports.PORT20)
    motor = vex.Motor29(expander.a)

    current_position = make_rotations_universal(encoder.position())
    print(current_position)
    if abs(current_position - required_position) < 5:
        motor.stop()
        print("AAAA")
        #vex.wait(10000)
    else:
        motor.spin(vex.FORWARD, 50, vex.PERCENT)
    

class VexSwerveModuleConstants():
    front_left_driving_port = 1
    front_left_turning_port = 1
    front_left_encoder_port = 1
    front_left_clockwise = True
    front_left_offset = 0

    front_right_driving_port = 2
    front_right_turning_port = 2
    front_right_encoder_port = 3
    front_right_clockwise = False
    front_right_offset = 0

    back_right_driving_port = 3
    back_right_turning_port = 4
    back_right_encoder_port = 4
    back_right_clockwise = False
    back_right_offset = 0

    back_left_driving_port = 4
    back_left_turning_port = 3
    back_left_encoder_port = 2
    back_left_clockwise = False
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

class VexSwerveModule():
    def __init__(self, driving_port, turning_port, encoder_port, clockwise, offset, id):
        self.driving_motor = vex.Motor(driving_port)
        self.turning_motor = vex.Motor29(turning_port)
        self.turning_encoder = vex.Encoder(encoder_port)
        self.clockwise = clockwise
        self.offset = offset
        self.id = id
        #calculates how many degrees the encoder can be off before it stops the motor.
        self.threshold = 2 # degrees
        #calculates how many degrees the encoder can be off before it starts to turn the wheel.
        self.threshold2 = 0 # degrees
        self.motion_dict = {'forward':vex.FORWARD, 'backward':vex.REVERSE}
        # Motor is overshooting. 
        self.angle_offset = 0

        #Thread Helper Vars
        #self.turning_motor_thread = vex.Thread(self.spin_all_turning_motors_permanent_thread, ())
        self.required_position = 90

        self.turning_encoder.set_position(90)

        #Check if motor was stopped
        self.stopped = False
        self.position = -1000

        #Slow down as x approaches a the desired angle
        # Graph generated from: https://www.desmos.com/calculator/sbtjfwfwmx. X represents angle currently at in degrees, y is speed in percent output
        #self.vel_compensation = lambda x: max(0, min(100, 22.74002 * x**(0.683772/2.03167) - 2.42683)) #<- Fast
        #self.vel_compensation = lambda x: max(0, min(100, 14.6363 * x**(-5644.21437/-13519.6038) - 1.2999)) #<- Slow
        self.vel_compensation = lambda x: max(0, min(100, 4.15943 * x**(1.2456/2.09046) - 2.5699)) #Super slow


    def get_state(self):
        return (rpm_to_mps(self.driving_motor.velocity()), make_rotations_universal(self.turning_encoder.position()))
    
    def get_position(self):
        return (self.driving_motor.position(), make_rotations_universal(self.turning_encoder.position()))
    
    def set_position(self, speed, rposition):

        # Adjust final position and speed to optimize movement.
        rposition=make_rotations_universal(rposition*VexSwerveModuleConstants.turning_gear_ratio/2) # REMEMBER THAT ROTATIONS ARE CALCULATED VIA THEIR THE SHAFT CONNECTD TO MOTOR, NOT WHEEL
        current_position = make_rotations_universal(self.turning_encoder.position())
        rposition, speed, offset = self.optimize(current_position, rposition, speed)
        rposition = make_rotations_universal(rposition)

        #self.driving_motor.spin(vex.FORWARD, speed*100, vex.PERCENT)

        self.required_position = [rposition][0]
        #if self.id == 3:
            #print(current_position, rposition)

        #if abs(make_rotations_universal(self.turning_encoder.position()) - rposition) > self.threshold and abs(rposition - self.turning_threads[-1][1]) > self.threshold2:
        #print(self.turning_encoder.velocity(), current_position, rposition, self.position)
        diff = check_if_angle_is_within_threshold(current_position, rposition, self.threshold)
        diff[1] = abs(diff[1])
        #print(self.turning_encoder.velocity(), current_position, rposition, self.position)
        if diff[0] and not (self.stopped and abs(self.turning_encoder.velocity()) > 1):
            self.position = -1000
            self.stopped = False
            self.turning_motor.stop()
            self.turning_motor.spin(
                        self.motion_dict[optimize_rotation(self.turning_encoder.position(), 
                                        self.required_position, 
                                        self.clockwise)
                                        ], 
                                        90, 
                                        vex.PERCENT)
        else:
            #print(self.turning_encoder.velocity(), current_position, rposition, self.position)
            #print(self.id)
            self.turning_motor.stop()
            #vex.wait(3000)
            #print("BBBBB")
            #print(self.turning_encoder.velocity(), current_position, rposition, self.position)
            current_position = make_rotations_universal(self.turning_encoder.position())
            self.turning_encoder.set_position(current_position)
            self.stopped = True
            if self.position == -1000:
                self.position = current_position
            if self.position != -1000:
                self.turning_encoder.set_position(self.position)
        #     if len(self.turning_threads) > 0:
        #         self.turning_motor.stop()
        #         self.turning_threads.pop(0)[0].stop()
        #     self.turning_threads.append((vex.Thread(self.spin_until_in_position, (rposition, vex.Timer())), rposition))
    
    # continues to run until the position is met
    def spin_until_in_position(self, position, start_time):
        self.turning_motor.stop()
        self.turning_motor.spin(
                self.motion_dict[optimize_rotation(self.turning_encoder.position(), 
                                position, 
                                self.clockwise)
                                ], 
                                100, 
                                vex.PERCENT)
        
        #counter = 1
        thresh = [self.threshold][0]
        while True:
            if abs(make_rotations_universal(self.turning_encoder.position()) - position) < thresh:
                self.turning_motor.stop()
                break
            if (start_time.time())/1000 > 0.5*counter: # may need to adjust counter
                thresh += 1
                counter += 1
            elif (start_time.time())/1000 > 2: #backup just in case. no memory errors plz
                self.turning_motor.stop()
                break
            vex.wait(1, vex.MSEC)
        return 

    def spin_all_turning_motors_permanent_thread(self):
        while True:
            if abs(make_rotations_universal(self.turning_encoder.position()) - self.required_position) > self.threshold:
                self.turning_motor.stop()
                self.turning_motor.spin(
                        self.motion_dict[optimize_rotation(self.turning_encoder.position(), 
                                        self.required_position, 
                                        self.clockwise)
                                        ], 
                                        100, 
                                        vex.PERCENT)
            thresh = [self.threshold][0]
            start_time = vex.Timer()
            counter = 1
            while True:
                if abs(make_rotations_universal(self.turning_encoder.position()) - self.required_position) < thresh:
                    #print(self.id)
                    self.turning_motor.stop()
                    break
                # if (start_time.time())/1000 > 0.5*counter: # may need to adjust counter
                #     thresh += 1
                #     counter += 1
                # elif (start_time.time())/1000 > 2: #backup just in case. no memory errors plz
                #     self.turning_motor.stop()
                #     break
                vex.wait(1, vex.MSEC)
            vex.wait(10, vex.MSEC)

    
    def reset_driving_encoder(self):
        self.driving_motor.reset_position()

    def reset_turning_encoder(self):
        self.turning_encoder.reset_position()

    def get_driving_vel(self):
        return self.driving_motor.velocity()
    
    def get_turning_vel(self):
        return self.turning_encoder.velocity()
    
    def optimize(self,current_angle, angle, speed):
        delta = angle - current_angle
        if abs(delta) > 90:
            speed *= -1
            angle = angle + 180 + self.angle_offset
            offset = self.angle_offset
        else:
            angle -= self.angle_offset
            offset = -self.angle_offset
        return angle, speed, offset


class DriveSubsystem():
    def __init__(self, brain):
        self.brain = brain
        self.expander = vex.Triport(vex.Ports.PORT20)
        driving_port_dict = {1: vex.Ports.PORT1, 2: vex.Ports.PORT2, 3: vex.Ports.PORT3, 4: vex.Ports.PORT4, 5: vex.Ports.PORT5, 6: vex.Ports.PORT6, 7: vex.Ports.PORT7, 8: vex.Ports.PORT8, 9: vex.Ports.PORT9, 10: vex.Ports.PORT10, 11: vex.Ports.PORT11, 12: vex.Ports.PORT12}
        turning_port_dict = {1:self.expander.a, 2:self.expander.b, 3:self.expander.c, 4:self.expander.d, 5:self.expander.e, 6:self.expander.f, 7:self.expander.g, 8: self.expander.h}
        encoder_port_dict = {1:brain.three_wire_port.a, 2:brain.three_wire_port.c, 3:brain.three_wire_port.e, 4:brain.three_wire_port.g}


        self.front_left_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.front_left_driving_port],
            turning_port_dict[VexSwerveModuleConstants.front_left_turning_port],
            encoder_port_dict[VexSwerveModuleConstants.front_left_encoder_port],
            VexSwerveModuleConstants.front_left_clockwise,
            VexSwerveModuleConstants.front_left_offset,
            VexSwerveModuleConstants.front_left_turning_port
        )
        self.front_right_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.front_right_driving_port],
            turning_port_dict[VexSwerveModuleConstants.front_right_turning_port],
            encoder_port_dict[VexSwerveModuleConstants.front_right_encoder_port],
            VexSwerveModuleConstants.front_right_clockwise,
            VexSwerveModuleConstants.front_right_offset,
            VexSwerveModuleConstants.front_right_turning_port
        )
        self.back_right_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.back_right_driving_port],
            turning_port_dict[VexSwerveModuleConstants.back_right_turning_port],
            encoder_port_dict[VexSwerveModuleConstants.back_right_encoder_port],
            VexSwerveModuleConstants.back_right_clockwise,
            VexSwerveModuleConstants.back_right_offset,
            VexSwerveModuleConstants.back_right_turning_port
        )
        self.back_left_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.back_left_driving_port],
            turning_port_dict[VexSwerveModuleConstants.back_left_turning_port],
            encoder_port_dict[VexSwerveModuleConstants.back_left_encoder_port],
            VexSwerveModuleConstants.back_left_clockwise,
            VexSwerveModuleConstants.back_right_offset,
            VexSwerveModuleConstants.back_left_turning_port
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
            angle = get_swerve_angle(x_final_speed, y_final_speed)
            module_desired_states.append((speed, angle))
        
        

        return module_desired_states
    
    def drive(self, xspeed, yspeed, rotspeed):
        module_states = self.chassisSpeedsToSwerveModuleStates(xspeed,yspeed,rotspeed)
        self.front_left_module.set_position(module_states[0][0], module_states[0][1])
        self.front_right_module.set_position(module_states[1][0], module_states[1][1])
        self.back_right_module.set_position(module_states[2][0], module_states[2][1])
        self.back_left_module.set_position(module_states[3][0], module_states[3][1])


# Brain should be defined by default
brain=vex.Brain()
drive_subsystem = DriveSubsystem(brain)
controller = vex.Controller()
count = 0
counter = 0

axes_list = [0, 0, 0]

if __name__ == "__main__":
    while True:
        # counter += 1
        # x3, x4, x1 = controller.axis3.position()/100, controller.axis4.position()/100, controller.axis1.position()/100
        drive_subsystem.drive(
            controller.axis3.position()/100,
            controller.axis4.position()/100,
            controller.axis1.position()/100
        # apply_deadband(controller.axis3.position(), 0.05),
        # apply_deadband(controller.axis4.position(), 0.05),
        # apply_deadband(controller.axis1.position(), 0.05),
        )
        # axes_list = [controller.axis3.position()/100, controller.axis4.position()/100, controller.axis1.position()/100]
        vex.wait(1, vex.MSEC)
        # print(brain.battery.current())

        
