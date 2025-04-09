# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       solkwak                                                      #
# 	Created:      4/9/2025, 9:33:39 AM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
from math_folder.swerve_utilities import *
from subsystems.DriveSubsystem import DriveSubsystem

# Brain should be defined by default
brain=Brain()
drive_subsystem = DriveSubsystem(brain)
controller = Controller()


if __name__ == "__main__":
    drive_subsystem.drive(
    apply_deadband(controller.axis3.position(), 0.05),
    apply_deadband(controller.axis4.position(), 0.05),
    apply_deadband(controller.axis1.position(), 0.05),
    )


        
