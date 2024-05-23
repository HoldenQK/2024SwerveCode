#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import wpilib
from wpilib.shuffleboard import Shuffleboard
from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

    #Work in progress for setting up shuffleboard for useful info in testing.
    # def robotPeriodic(self) -> None:
    #     wpilib.SmartDashboard.putNumber("Strafe", self.container.)
    #     wpilib.SmartDashboard.putBoolean("", )
    #     wpilib.SmartDashboard.putBoolean("", )
    #     wpilib.SmartDashboard.putBoolean("", )

    def teleopInit(self) -> None:
        self.container.__init__()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)
