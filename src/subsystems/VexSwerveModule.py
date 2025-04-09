import vex
import threading
import time
from math_folder.swerve_utilities import *
from constants import *

class VexSwerveModule():
    def __init__(self, driving_port, turning_port, encoder_port, clockwise, offset):
        self.driving_motor = vex.Motor(driving_port)
        self.turning_motor = vex.Motor29(turning_port)
        self.turning_encoder = vex.Encoder(encoder_port)
        self.clockwise = clockwise
        self.offset = offset

        self.motion_dict = {'forward':vex.FORWARD, 'backward':vex.REVERSE}

    def get_state(self):
        return (rpm_to_mps(self.driving_motor.velocity()), make_rotations_universal(self.turning_encoder.position()))
    
    def get_position(self):
        return (self.driving_motor.position(), make_rotations_universal(self.turning_encoder.position()))
    
    def set_position(self, speed, rposition):
        self.driving_motor.spin(vex.FORWARD, mps_to_rpm(speed), vex.RPM)
        self.turning_motor.spin(
            self.motion_dict[optimize_rotation(self.turning_encoder.position(), 
                              rposition, 
                              self.clockwise)], 
                              degps_to_rpm(VexSwerveModuleConstants.turning_max_speed), 
                              vex.RPM)
        start_time = time.time()
        turning_thread = threading.Thread(target=self.spin_turning_position, args=(rposition, start_time), daemon=True)
        turning_thread.start()
    
    # continues to run until the position is met
    def spin_turning_position(self, position, start_time):
        while True:
            if abs(make_rotations_universal(self.turning_encoder.position()) - position) < 1:
                self.turning_motor.stop()
                break
            if time.time() - start_time > 2: #backup just in case. no memory errors plz
                self.turning_motor.stop()
                break
    
    def reset_driving_encoder(self):
        self.driving_motor.reset_position()

    def reset_turning_encoder(self):
        self.turning_encoder.reset_position()

    def get_driving_vel(self):
        return self.driving_motor.velocity()
    
    def get_turning_vel(self):
        return self.turning_encoder.velocity()