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
import typing
import math

import romi
import wpilib
import wpilib.drive
import commands2

from utils.signalLogging import SignalWrangler
from utils.signalLogging import log
from utils.crashLogger import CrashLogger
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.robotIdentification import RobotIdentification

# Uncomment these lines and set the port to the pycharm debugger to use the
# Pycharm debug server to debug this code.

#import pydevd_pycharm
#pydevd_pycharm.settrace('localhost', port=61890, stdoutToServer=True, stderrToServer=True)

# If your ROMI isn't at the default address, set that here
os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"


class AutoState1():
    def __init__(self, robot, option_number):
        self.robot = robot
        self.state = 'start'
        self.entered_state_time = 0
        self.option_number = option_number

        self.states = {
            'start': 1,
            'drive': 2,
            'stop': 3,
        }

    def start(self):
        self.state = 'start'

    def periodic(self):
        now = wpilib.Timer.getFPGATimestamp()
        match self.state:
            case 'start':
                self.entered_state_time = wpilib.Timer.getFPGATimestamp()
                self.state = 'drive'
            case 'drive':
                if now-self.entered_state_time>1.0:
                    self.state = 'stop'
                    self.robot.drive.arcadeDrive(0.0, 0)
                else:
                    self.robot.drive.arcadeDrive(0.5, 0)
            case 'stop' | _:
                self.robot.drive.arcadeDrive(0, 0)
        log("autostate", self.state_to_int(), "int")

    def state_to_int(self):
        result = -1
        if self.state in self.states:
            result = self.states[self.state]
        return result


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None
    kCountsPerRevolution = 1440.0
    kWheelDiameterInch = 2.75591
    def robotInit(self) -> None:

        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init

        self.onboardIO = romi.OnBoardIO(
            romi.OnBoardIO.ChannelMode.INPUT, romi.OnBoardIO.ChannelMode.INPUT
        )

        # Assumes a gamepad plugged into channnel 0
        self.controller = wpilib.Joystick(0)

        # Create SmartDashboard chooser for autonomous routines

        self.chooser = wpilib.SendableChooser()
        self.chooser.setDefaultOption(
            "Auto Routine 1",AutoState1(self,1)
        )
        self.chooser.addOption("Auto Routine 2", AutoState1(self,2))
        self.chooser.addOption("Auto Routine 3", AutoState1(self,3))
        wpilib.SmartDashboard.putData("Auto choices", self.chooser)


        # Example of how to use the onboard IO
        onboardButtonA = commands2.button.Trigger(self.onboardIO.getButtonAPressed)
        onboardButtonA.onTrue(commands2.PrintCommand("Button A Pressed")).onFalse(
            commands2.PrintCommand("Button A Released")
        )


        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = wpilib.Spark(0)
        self.leftMotor.setInverted(True)
        self.rightMotor = wpilib.Spark(1)
        self.rightMotor.setInverted(False)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)

        self.rightEncoder = wpilib.Encoder(6, 7)

        # Set up the differential drive controller
        self.drive = wpilib.drive.DifferentialDrive(self.leftMotor, self.rightMotor)

        # Set up the RomiGyro
        self.gyro = romi.RomiGyro()

        # Set up the BuiltInAccelerometer
        self.accelerometer = wpilib.BuiltInAccelerometer()

        self.resetEncoders()

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (math.pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )

        self.rId = RobotIdentification()
        self.crashLogger = CrashLogger()
        self.stt = SegmentTimeTracker()


    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        leftEncoderCount = self.rightEncoder.get()
        rightEncoderCount = self.leftEncoder.get()
        log("driveLeftEncoderCount", leftEncoderCount, "counts")
        log("driveRightEncoderCount", rightEncoderCount, "counts")
        leftEncoderCount = self.rightEncoder.get()
        rightEncoderCount = self.leftEncoder.get()
        log("driveLeftEncoderDistance", self.getLeftDriveDistanceInches(), "inches")
        log("driveRightEncoderDistance",self.getRightDriveDistanceInches(), "inches")
        x = self.getGyroAngleX()
        y = self.getGyroAngleY()
        z = self.getGyroAngleZ()
        log("driveGyroAngleX", x, "deg")
        log("driveGyroAngleY", y, "deg")
        log("driveGyroAngleZ", z, "deg")

        if self.autonomousCommand is not None:
            log("autonomousCommand", self.autonomousCommand.option_number, "int")

        SignalWrangler().publishPeriodic()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.chooser.getSelected()
        self.autonomousCommand.start()


    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        self.autonomousCommand.periodic()


    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        pass

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        forward = self.forward()
        rotation = self.rotation()

        # we might have some confusion: In some modules Drive is: wpilib.drive.DifferentialDrive
        # in some modules drive is: drivetrain.
        self.drive.arcadeDrive(self.forward(), self.rotation())

        log("driveForwardCmd", forward, "ratio")
        log("driveRotationCmd", rotation, "ratio")
        log( "leftOutputPWM", self.leftMotor.get(), "ratio")
        log( "leftOutputVolts", self.leftMotor.getVoltage(), "V")
        log( "rightOutputPWM", self.rightMotor.get(), "ratio")
        log( "rightOutputVolts", self.rightMotor.getVoltage(), "V")

    def testInit(self) -> None:
        pass

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def forward(self):
        return -self.controller.getRawAxis(1)

    def rotation(self):
        return self.controller.getRawAxis(4)

    def getAccelX(self) -> float:
        """The acceleration in the X-axis.

        :returns: The acceleration of the Romi along the X-axis in Gs
        """
        return self.accelerometer.getX()

    def getAccelY(self) -> float:
        """The acceleration in the Y-axis.

        :returns: The acceleration of the Romi along the Y-axis in Gs
        """
        return self.accelerometer.getY()

    def getAccelZ(self) -> float:
        """The acceleration in the Z-axis.

        :returns: The acceleration of the Romi along the Z-axis in Gs
        """
        return self.accelerometer.getZ()

    def getGyroAngleX(self) -> float:
        """Current angle of the Romi around the X-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleX()

    def getGyroAngleY(self) -> float:
        """Current angle of the Romi around the Y-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleY()

    def getGyroAngleZ(self) -> float:
        """Current angle of the Romi around the Z-axis.

        :returns: The current angle of the Romi in degrees
        """
        return self.gyro.getAngleZ()

    def getLeftDriveDistanceInches(self):
        return -self.leftEncoder.getDistance()


    def getRightDriveDistanceInches(self):
        return -self.rightEncoder.getDistance()


