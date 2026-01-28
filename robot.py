#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# Example that shows how to connect to a ROMI from RobotPy
#
# Requirements
# ------------
#
#    # Install https://github.com/wpilibsuite/WPILibPi/releases/download/v2023.2.1/WPILibPi_64_image-v2023.2.1-Romi.zip
#    # on your Raspberry Pi sd card.
#
#    # On Windows, some people prefer to run python 3
#    py -3
#
#    # but sometimes when using Python virtual environments (venv) "py -3" does not run the python associated with
#    # the virtual environment, some people (this author) avoids "py -3", preferring "python"
#
#    # confirm that your python is 3.12 or greater
#    python -VV
#
#    python -m pip install robotpy
#    python -m pip install robotpy-halsim-ws
#
#
# Run the program
# ---------------
#
# To run the program you will need to explicitly use the ws-client option:
#
#    cd to this directory
#    python -m robotpy sync
#
#    power-up the Romi
#    connect to a WiFi network where the romi is on.
#
#    python -m robotpy sim --ws-client
#
# By default the WPILib simulation GUI will be displayed. To disable the display
# you can add the --nogui option
#

import os
from typing import Optional
import wpilib.drive
from wpilib.deployinfo import getDeployData
from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger

from commands2 import cmd, CommandScheduler, Command, PrintCommand

import constants

from wpilib import RobotBase
from robotcontainer import RobotContainer



# Uncomment these lines and set the port to the pycharm debugger to use the
# Pycharm debug server to debug this code.

#import pydevd_pycharm
#pydevd_pycharm.settrace('localhost', port=61890, stdoutToServer=True, stderrToServer=True)

# If your ROMI isn't at the default address, set that here
os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"




class MyRobot(LoggedRobot):

    activeCommand: Optional[Command] = None
    # kCountsPerRevolution = 1440.0
    # kWheelDiameterInch = 2.75591

    def __init__(self) -> None:
        super().__init__()
        Logger.recordMetadata("Robot", "Romi")

        print(f"isRealRomiMode()={constants.isRealRomiMode()} RobotBase.isReal()={RobotBase.isReal()} constants.kRobotMode={constants.kRobotMode}")

        match constants.kRobotMode:
            case constants.RobotModes.REAL|constants.RobotModes.SIMULATION:
                deploy_config = getDeployData()
                if deploy_config is not None:
                    Logger.recordMetadata(
                        "Deploy Host", deploy_config.get("deploy-host", "")
                    )
                    Logger.recordMetadata(
                        "Deploy User", deploy_config.get("deploy-user", "")
                    )
                    Logger.recordMetadata(
                        "Deploy Date", deploy_config.get("deploy-date", "")
                    )
                    Logger.recordMetadata(
                        "Code Path", deploy_config.get("code-path", "")
                    )
                    Logger.recordMetadata("Git Hash", deploy_config.get("git-hash", ""))
                    Logger.recordMetadata(
                        "Git Branch", deploy_config.get("git-branch", "")
                    )
                    Logger.recordMetadata(
                        "Git Description", deploy_config.get("git-desc", "")
                    )
                Logger.addDataReciever(NT4Publisher(True))
                Logger.addDataReciever(WPILOGWriter())
            case constants.RobotModes.REPLAY:
                self.useTiming = False  # run as fast as possible
                self.useTiming = True  # Mike Stitt added this
                log_path = os.environ["LOG_PATH"]
                log_path = os.path.abspath(log_path)
                print(f"Starting log from {log_path}")
                Logger.setReplaySource(WPILOGReader(log_path))
                Logger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))
                Logger.addDataReciever(NT4Publisher(True))  # Mike Stitt added this

        Logger.start()

        self.robotContainer = RobotContainer()

    def robotInit(self) -> None:

        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        pass




    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        # Runs the Scheduler. This is responsible for polling buttons, adding
        # newly-scheduled commands, running already-scheduled commands, removing
        # finished or interrupted commands, and running subsystem periodic() methods.
        # This must be called from the robot's periodic block in order for anything in
        # the Command-based framework to work.
        CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        CommandScheduler.getInstance().cancelAll()

        self.activeCommand = self.robotContainer.getAutonomousCommand()

        if self.activeCommand is not None:
            CommandScheduler.getInstance().schedule(self.activeCommand)

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def autonomousExit(self):
        """This function is called after autonomous command is executed"""
        if self.activeCommand is not None:
            self.activeCommand.cancel()
            self.activeCommand = None


    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        CommandScheduler.getInstance().cancelAll()
        self.activeCommand = None
        self.robotContainer.configureButtonBindingsOpenLoop()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

        self.activeCommand = self.robotContainer.getTestCommand()

        if self.activeCommand is not None:
            CommandScheduler.getInstance().schedule(self.activeCommand)


    def testPeriodic(self) -> None:
        pass

    def testExit(self) -> None:
        """This function is called after test is executed"""
        if self.activeCommand is not None:
            self.activeCommand.cancel()
            self.activeCommand = None


    def simulationInit(self) -> None:
        pass

    def simulationPeriodic(self) -> None:
        pass

    def endCompetition(self) -> None:
        super().endCompetition()


if __name__ == "__main__":
    wpilib.run(MyRobot)
