import vex
from VexSwerveModule import VexSwerveModule
from constants import *
from math_folder.swerve_utilities import *
class DriveSubsystem():
    def __init__(self, brain):
        self.brain = brain
        driving_port_dict = {1: vex.Ports.PORT1, 2: vex.Ports.PORT2, 3: vex.Ports.PORT3, 4: vex.Ports.PORT4, 5: vex.Ports.PORT5, 6: vex.Ports.PORT6, 7: vex.Ports.PORT7, 8: vex.Ports.PORT8, 9: vex.Ports.PORT9, 10: vex.Ports.PORT10, 11: vex.Ports.PORT11, 12: vex.Ports.PORT12}
        turning_port_dict = {1:brain.three_wire_port.a, 2:brain.three_wire_port.b, 3:brain.three_wire_port.c, 4:brain.three_wire_port.d, 5:brain.three_wire_port.e, 6:brain.three_wire_port.f, 7:brain.three_wire_port.g, 8:brain.three_wire_port.h}
        
        self.front_left_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.front_left_driving_port],
            turning_port_dict[VexSwerveModuleConstants.front_left_turning_port],
            turning_port_dict[VexSwerveModuleConstants.front_left_encoder_port],
            VexSwerveModuleConstants.front_left_clockwise,
            VexSwerveModuleConstants.front_left_offset
        )
        self.front_right_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.front_right_driving_port],
            turning_port_dict[VexSwerveModuleConstants.front_right_turning_port],
            turning_port_dict[VexSwerveModuleConstants.front_right_encoder_port],
            VexSwerveModuleConstants.front_right_clockwise,
            VexSwerveModuleConstants.front_right_offset
        )
        self.back_right_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.back_right_driving_port],
            turning_port_dict[VexSwerveModuleConstants.back_right_turning_port],
            turning_port_dict[VexSwerveModuleConstants.back_right_encoder_port],
            VexSwerveModuleConstants.back_right_clockwise,
            VexSwerveModuleConstants.back_right_offset
        )
        self.back_left_module = VexSwerveModule(
            driving_port_dict[VexSwerveModuleConstants.back_left_driving_port],
            turning_port_dict[VexSwerveModuleConstants.back_left_turning_port],
            turning_port_dict[VexSwerveModuleConstants.back_left_encoder_port],
            VexSwerveModuleConstants.back_left_clockwise,
            VexSwerveModuleConstants.back_right_offset
        )

    def chassisSpeedsToSwerveModuleStates(self, xspeed, yspeed, rotspeed):
        inverse_kinematics = DrivingConstants.inverse_kinematics
            
        module_final_states = multiply_matrices(inverse_kinematics, [[0],[0],[xspeed],[yspeed],[rotspeed]])
        module_desired_states = []

        for i in range(len(DrivingConstants.drive_kinematics)):
            x_final_speed = module_final_states[2*i]
            y_final_speed = module_final_states[2*i+1]
            # inefficient. get rid of it after implementing desaturate wheelSpeeds
            speed = max(VexSwerveModuleConstants.driving_max_speed, (x_final_speed**2 + y_final_speed**2)**(1/2))
            angle = get_swerve_angle(x_final_speed, y_final_speed)
            module_desired_states.append((speed, angle))

        return module_desired_states
    
    def drive(self, xspeed, yspeed, rotspeed):
        module_states = self.chassisSpeedsToSwerveModuleStates(xspeed,yspeed,rotspeed)
        
        self.front_left_module.set_position(module_states[0][0], module_states[0][1])
        self.front_right_module.set_position(module_states[1][0], module_states[1][1])
        self.back_right_module.set_position(module_states[2][0], module_states[2][1])
        self.back_left_module.set_position(module_states[3][0], module_states[3][1])