from commands2 import cmd, Command
from wpilib import SendableChooser, SmartDashboard

from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from pykit.loggedrobot import LoggedRobot

class MyRobot(LoggedRobot):

    def __init__(self) -> None:
        super().__init__()

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


