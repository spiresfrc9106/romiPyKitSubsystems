import os
from typing import Optional
from commands2 import Command, cmd
from commands2.sysid import SysIdRoutine
from commands2.button import CommandXboxController
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from pathplannerlib.auto import AutoBuilder, NamedCommands
from wpilib import getDeployDirectory, XboxController, SendableChooser, SmartDashboard

from commands.drivecommands import DriveCommands
from subsystems.drive.drive import Drive
from subsystems.drive.driveio import DriveIO
from subsystems.drive.driveiosim import DriveIOSim
from subsystems.drive.driveiotalonfx import DriveIOTalonFX
from subsystems.drive.driveioromispark import DriveIORomiSpark

from subsystems.drive.gyroio import GyroIO
from subsystems.drive.gyroiopigeon2 import GyroIOPigeon2
from subsystems.drive.gyroioromi import GyroIORomi
from wpimath import applyDeadband

import constants


class RobotContainer:
    def __init__(self) -> None:
        match constants.kRobotMode:
            case constants.RobotModes.REAL:
                self.drive = Drive(DriveIORomiSpark(), GyroIORomi())
                #self.drive = Drive(DriveIORomiSpark(), GyroIOPigeon2())
            case constants.RobotModes.SIMULATION:
                self.drive = Drive(DriveIOSim(), GyroIO())
            case constants.RobotModes.REPLAY:
                self.drive = Drive(DriveIO(), GyroIO())

        auto_folder_path = os.path.join(getDeployDirectory(), "pathplanner", "autos")
        auto_list = os.listdir(auto_folder_path)


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
        for auto in auto_list:
            auto = auto.removesuffix(".auto")
            self.autoChooser.addOption(
                auto,
                AutoBuilder.buildAuto(auto),
            )
        self.autoChooser.setDefaultOption("Auto Do Nothing", cmd.none())

        self.autoSendableChooser = SendableChooser()
        self.autoSendableChooser.addOption("Option 1 - Sendable Auto Choices", cmd.none())
        self.autoSendableChooser.setDefaultOption("Sendable Auto Do Nothing", cmd.none())

        SmartDashboard.putData("Sendable Auto Choices", self.autoSendableChooser)

        self.controller = XboxController(0)


        self.testChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser(
            "Test Choices"
        )

        ffWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        ffCmd = DriveCommands.feedForwardCharacterization(self.drive)
        ffWithWaits = ffWaitCmd.andThen(ffCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Test - Drive Simple FF Characterization",
            ffWithWaits
        )

        sysIdQFWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        sysIdQFCmd = self.drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        sysIdQFWithWaits = sysIdQFWaitCmd.andThen(sysIdQFCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Test - Drive SysId (Quasistatic Forward)",
            sysIdQFWithWaits
        )

        sysIdQRWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        sysIdQRCmd = self.drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        sysIdQRWithWaits = sysIdQRWaitCmd.andThen(sysIdQRCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Test - Drive SysId (Quasistatic Reverse)",
            sysIdQRWithWaits
        )

        sysIdDFWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        sysIdDFCmd = self.drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        sysIdDFWithWaits = sysIdDFWaitCmd.andThen(sysIdDFCmd.onlyWhile(lambda: self.controller.getRightBumper()))

        self.testChooser.addOption(
            "Test - Drive SysId (Dynamic Forward)",
            sysIdDFWithWaits
        )

        sysIdDRWaitCmd = cmd.waitUntil(lambda: self.controller.getRightBumper())
        sysIdDRCmd = self.drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        sysIdDRWithWaits = sysIdDRWaitCmd.andThen(sysIdDRCmd.onlyWhile(lambda: self.controller.getRightBumper()))


        self.testChooser.addOption(
            "Test - Drive SysId (Dynamic Reverse)",
            sysIdDRWithWaits
        )

        self.testChooser.setDefaultOption("Test Do Nothing", cmd.none())

        self.testSendableChooser = SendableChooser()
        self.testSendableChooser.addOption("Option 1 - Sendable Test Choices", cmd.none())
        self.testSendableChooser.setDefaultOption("Sendable Test Do Nothing", cmd.none())

        SmartDashboard.putData("Sendable Test Choices", self.testSendableChooser)


        self.configureButtonBindingsNone()

    def configureButtonBindingsNone(self) -> None:
        self.drive.setDefaultCommand(
            DriveCommands.arcadeDriveOpenLoop(
                self.drive,
                lambda: 0.0,
                lambda: 0.0,
                lambda: 0.25
            )
        )

    def configureButtonBindingsClosedLoop(self) -> None:
        self.drive.setDefaultCommand(
            DriveCommands.arcadeDriveClosedLoop(
                self.drive,
                lambda: -self.controller.getLeftY(),
                lambda: -self.controller.getRightX(),
                lambda: 1.0 if (self.controller.getRightBumper()) else 0.25,
            )
        )

    def configureButtonBindingsOpenLoop(self) -> None:
        self.drive.setDefaultCommand(
            DriveCommands.arcadeDriveOpenLoop(
                self.drive,
                lambda: -self.controller.getLeftY(),
                lambda: -self.controller.getRightX(),
                lambda: 1.0 if (self.controller.getRightBumper()) else 0.25,
            )
        )


    def getAutonomousCommand(self) -> Optional[Command]:
        return self.autoChooser.getSelected()

    def getTestCommand(self) -> Optional[Command]:
        return self.testChooser.getSelected()

