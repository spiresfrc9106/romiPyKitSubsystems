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
from math import pi

import romi
import wpilib
from wpilib.drive import DifferentialDrive
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.geometry import (
    Rotation2d,
    Pose2d,
)
from wpimath.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics,
    DifferentialDriveWheelSpeeds,
)
from commands2 import (
    Command,
    TimedCommandRobot,
)

from utils.signalLogging import SignalWrangler
from utils.signalLogging import log
from utils.helpfulmath import deadband

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
        speeds = DifferentialDrive.WheelSpeeds()
        match self.state:
            case 'start':
                self.entered_state_time = wpilib.Timer.getFPGATimestamp()
                self.state = 'drive'
                speeds = DifferentialDrive.arcadeDriveIK(0, 0, False)
            case 'drive':
                if now-self.entered_state_time>1.0:
                    self.state = 'stop'
                    speeds = DifferentialDrive.arcadeDriveIK(0, 0, False)
                else:
                    speeds = DifferentialDrive.arcadeDriveIK(0.5, 0, False)
            case 'stop' | _:
                speeds = DifferentialDrive.arcadeDriveIK(0, 0, False)
        self.robot.setMotorSpeeds(speeds)

        log("autostate", self.state_to_int(), "int")

    def state_to_int(self):
        result = -1
        if self.state in self.states:
            result = self.states[self.state]
        return result


class MyRobot(TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[Command] = None


    kInchesPerFoot = 12.0
    kCmsPerMeter = 100.0
    kCmPerInch = 2.540_000
    kMetersPerInch = kCmPerInch / kCmsPerMeter
    kRadiansPerRevolution = 2 * pi
    kDegreesPerRevolution = 360.0
    kRadiansPerDegree = kRadiansPerRevolution / kDegreesPerRevolution

    kCountsPerRevolution = 1440.0
    kWheelDiameterInch = 2.75591
    kWheelRadiusM = (kWheelDiameterInch / 2.0) * kMetersPerInch
    kTrackWidthM = 5.5 * kMetersPerInch


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
        self.chooser.addOption("Auto Routine 2 - same as 1", AutoState1(self,2))
        wpilib.SmartDashboard.putData("Auto choices", self.chooser)

        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = wpilib.Spark(0)
        self.leftMotor.setInverted(False)
        self.rightMotor = wpilib.Spark(1)
        self.rightMotor.setInverted(True)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)

        self.rightEncoder = wpilib.Encoder(6, 7)

        self.resetEncoders()


        # Set up the differential drive controller
        #self.drive = wpilib.drive.DifferentialDrive(self.leftMotor, self.rightMotor)

        # Set up the RomiGyro
        self.gyro = romi.RomiGyro()



        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )


        driveLeftEncoderDistanceInches =  self.getLeftDriveDistanceInches()
        driveRightEncoderDistanceInches = self.getRightDriveDistanceInches()

        log("driveLeftEncoderDistanceInches", driveLeftEncoderDistanceInches, "inches")
        log("driveRightEncoderDistanceInches",driveLeftEncoderDistanceInches, "inches")

        self.lastLeftEncoderDistanceM  = driveLeftEncoderDistanceInches * self.kMetersPerInch
        self.lastRightEncoderDistanceM = driveRightEncoderDistanceInches * self.kMetersPerInch

        self.kinematics = DifferentialDriveKinematics(self.kTrackWidthM)

        self.poseEstimator = DifferentialDrivePoseEstimator(
            self.kinematics, Rotation2d(), 0.0, 0.0, Pose2d()
        )
        self.rawGyroRotation = Rotation2d()

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        leftEncoderCount = self.rightEncoder.get()
        rightEncoderCount = self.leftEncoder.get()
        log("driveLeftEncoderCount", leftEncoderCount, "counts")
        log("driveRightEncoderCount", rightEncoderCount, "counts")

        driveLeftEncoderDistanceInches =  self.getLeftDriveDistanceInches()
        driveRightEncoderDistanceInches = self.getRightDriveDistanceInches()

        log("driveLeftEncoderDistanceInches", driveLeftEncoderDistanceInches, "inches")
        log("driveRightEncoderDistanceInches",driveLeftEncoderDistanceInches, "inches")

        leftEncoderDistanceM = driveLeftEncoderDistanceInches * self.kMetersPerInch
        rightEncoderDistanceM = driveRightEncoderDistanceInches * self.kMetersPerInch


        self.yawPositionDeg = -self.gyro.getAngle() / self.kRadiansPerDegree
        self.yawVelocityDegPerSec = -self.gyro.getRate() / self.kRadiansPerDegree
        self.yawPosition = Rotation2d.fromDegrees(self.yawPositionDeg)

        log("driveGyroYaw", self.yawPositionDeg, "deg")

        twist = self.kinematics.toTwist2d(
            leftEncoderDistanceM - self.lastLeftEncoderDistanceM,
            rightEncoderDistanceM - self.lastRightEncoderDistanceM,
        )

        # TODO This seems misleading it's called rawGyroRotation,
        #  but it seems entirely based upon wheel odometry
        self.rawGyroRotation = self.rawGyroRotation + Rotation2d(twist.dtheta)

        self.lastLeftEncoderDistanceM = leftEncoderDistanceM
        self.lastRightEncoderDistanceM = rightEncoderDistanceM

        self.poseEstimator.update(
            self.rawGyroRotation, leftEncoderDistanceM, rightEncoderDistanceM
        )

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
        rawForward = self.forward()
        rawRotation = self.rotation()

        forward = deadband(rawForward, 0.1) * self.slowMultiplier()
        rotation = deadband(rawRotation, 0.1) * self.slowMultiplier()

        speeds = DifferentialDrive.arcadeDriveIK(forward, rotation, False)
        self.setMotorSpeeds(speeds)

        log("driveForwardCmd", rawForward, "ratio")
        log("driveRotationCmd", rawRotation, "ratio")

    def testInit(self) -> None:
        pass

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def forward(self):
        return -self.controller.getRawAxis(1)

    def rotation(self):
        return -self.controller.getRawAxis(4)

    def slowMultiplier(self):
        return 1.0 if (self.controller.getRawButton(6)) else 0.25

    def getLeftDriveDistanceInches(self):
        return -self.leftEncoder.getDistance()


    def getRightDriveDistanceInches(self):
        return -self.rightEncoder.getDistance()

    def setMotorSpeeds(self, speeds):
        self.leftMotor.setVoltage(speeds.left*12)
        self.rightMotor.setVoltage(speeds.right*12)


