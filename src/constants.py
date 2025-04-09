class VexSwerveModuleConstants():
    front_left_driving_port = 1
    front_left_turning_port = 1
    front_left_encoder_port = 1
    front_left_clockwise = True
    front_left_offset = 0

    front_right_driving_port = 1
    front_right_turning_port = 1
    front_right_encoder_port = 1
    front_right_clockwise = True
    front_right_offset = 0

    back_right_driving_port = 1
    back_right_turning_port = 1
    back_right_encoder_port = 1
    back_right_clockwise = True
    back_right_offset = 0

    back_left_driving_port = 1
    back_left_turning_port = 1
    back_left_encoder_port = 1
    back_left_clockwise = True
    back_left_offset = 0

    wheel_radius = 0 # in meters
    turning_max_speed = 960 # deg/s
    driving_max_speed = 1 # m/s

class DrivingConstants():
    '''
    See Derivation of Inverse Kinematics for Swerve.pdf for a brilliant explanation of the inverse kinematics derivation
    https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
    '''

    wheel_base = 0
    track_width = 0

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