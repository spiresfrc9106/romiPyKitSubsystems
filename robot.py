#!/usr/bin/env python3

from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser



import os
from typing import Optional
import wpilib.drive
from wpilib.deployinfo import getDeployData
from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger

from commands2 import cmd, CommandScheduler, Command, PrintCommand

import constants

from wpilib import RobotBase, SendableChooser, SmartDashboard


# If your ROMI isn't at the default address, set that here
os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"




class MyRobot(LoggedRobot):

    activeCommand: Optional[Command] = None


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

        """
        For debugging purposes, this code makes 4 choosers
        - self.autoChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(...
        - self.autoSendableChooser = SendableChooser()
        - self.testChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(...
        - self.testSendableChooser = SendableChooser()

        When running the code it seems, like a bug. The menu items for self.autoChooser are
        placed in the self.testChooser. But the self.autoSendableChooser and 
        self.testSendableChooser
        are each able to maintain independent lists of options.

        To run this code:
        clone this repo
        checkout this commit
        uv sync
        uv run -- robotpy sync
        uv run -- robotpy sim

        -Mike Stitt 2026-01-28

        """
        self.autoChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Auto Choices"
        )

        self.autoChooser.addOption("Auto Option 1", cmd.none())
        self.autoChooser.addOption("Auto Option 2", cmd.none())
        self.autoChooser.setDefaultOption("Auto Do Nothing", cmd.none())

        self.autoSendableChooser = SendableChooser()
        self.autoSendableChooser.addOption("Option 1 - Sendable Auto Choices", cmd.none())
        self.autoSendableChooser.setDefaultOption("Sendable Auto Do Nothing", cmd.none())

        SmartDashboard.putData("Sendable Auto Choices", self.autoSendableChooser)

        self.testChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Test Choices"
        )

        self.testChooser.addOption(
            "Test - Option 1",
            cmd.none()
        )

        self.testChooser.addOption(
            "Test - Option 2",
            cmd.none()
        )

        self.testChooser.setDefaultOption("Test Do Nothing", cmd.none())

        self.testSendableChooser = SendableChooser()
        self.testSendableChooser.addOption("Option 1 - Sendable Test Choices", cmd.none())
        self.testSendableChooser.setDefaultOption("Sendable Test Do Nothing", cmd.none())

        SmartDashboard.putData("Sendable Test Choices", self.testSendableChooser)

        Logger.start()


    def robotInit(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

        self.activeCommand = self.autoChooser.getSelected()

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

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()

        self.activeCommand = self.testChooser.getSelected()

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
