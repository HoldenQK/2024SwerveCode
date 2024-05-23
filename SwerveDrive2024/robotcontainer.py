import math

import commands2
import commands2.button
import commands2.subsystem
import wpimath
import wpilib

from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

#from arm_pid_to_position import ArmPIDToPosition
#from intake_in import IntakeIn
#from intake_out import IntakeOut
# from stop_arm_and_wrist import StopArmAndWrist
# from wrist_pid_to_position import WristPIDToPosition
from constants import AutoConstants, DriveConstants, OIConstants
# from arm_subsystem import ArmSubsystem
# from intake_subsystem import IntakeSubsystem
# from wrist_subsystem import WristSubsystem
from drivesubsystem import DriveSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # The driver controllers
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)
        self.operatorController = wpilib.XboxController(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        (self.driverController.getLeftY() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getLeftX() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))),
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        (-self.driverController.getLeftY() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getLeftX() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))),
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    True,
                    False,
                ),
                [self.robotDrive],
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Set wheels to X (brake)
        commands2.button.JoystickButton(self.driverController, 7).toggleOnTrue(
            commands2.RunCommand(
                lambda: self.robotDrive.setX(),
                [self.robotDrive],
            )
        )

        # Slow mode
        commands2.button.JoystickButton(self.driverController, 6).toggleOnTrue(
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        ((self.driverController.getLeftY() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getLeftX() * math.cos(self.robotDrive.getHeading() * (math.pi / 180)))) * 0.5,
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        ((-self.driverController.getLeftY() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getLeftX() * math.sin(self.robotDrive.getHeading() * (math.pi / 180)))) * 0.5,
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX() * 0.5, OIConstants.kDriveDeadband
                    ),
                    True,
                    False,
                ),
                [self.robotDrive],
            )
        )


    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

