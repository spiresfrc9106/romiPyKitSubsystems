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
from dataclasses import dataclass, field

import romi
import wpilib
from wpilib import (
    Encoder,
    RobotBase,
    Spark,
)
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
from wpilib.simulation import DifferentialDrivetrainSim
from wpimath.system.plant import DCMotor

from commands2 import (
    Command,
    TimedCommandRobot,
)
from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.autolog import (
    autolog,
    autolog_output,
    autologgable_output,
)
from pykit.loggedrobot import LoggedRobot
from pykit.logger import Logger


from commands2 import CommandScheduler, Command, cmd

from commands2.button import Trigger

import constants
from utils.signalLogging import SignalWrangler
from utils.signalLogging import log
from utils.helpfulmath import deadband
from utils.helpfulmath import clamp

# Uncomment these lines and set the port to the pycharm debugger to use the
# Pycharm debug server to debug this code.

#import pydevd_pycharm
#pydevd_pycharm.settrace('localhost', port=61890, stdoutToServer=True, stderrToServer=True)

# If your ROMI isn't at the default address, set that here
os.environ["HALSIMWS_HOST"] = "10.0.0.2"
os.environ["HALSIMWS_PORT"] = "3300"


class AutoDriveForward1Sec():
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


    def getCommand(self) -> Command:
        def run():
            now = wpilib.Timer.getFPGATimestamp()
            speeds = DifferentialDrive.WheelSpeeds()
            match self.state:
                case 'start':
                    self.entered_state_time = wpilib.Timer.getFPGATimestamp()
                    self.state = 'drive'
                    self.robot.setForwardAndRotationCommands(0.0, 0.0)
                case 'drive':
                    if now-self.entered_state_time>1.0:
                        self.state = 'stop'
                        self.robot.setForwardAndRotationCommands(0.0, 0.0)
                    else:
                        self.robot.setForwardAndRotationCommands(0.5, 0.0)
                case 'stop' | _:
                    self.robot.setForwardAndRotationCommands(0.0, 0.0)

            log("autostate", self.state_to_int(), "int")

        result = cmd.run(run, None).withName(f"AutoDriveForward1Sec[{self.option_number}]")
        return result

    def state_to_int(self):
        result = -1
        if self.state in self.states:
            result = self.states[self.state]
        return result

@autologgable_output
class MyRobot(LoggedRobot):
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
    kMotorReduction = 10.71
    kGearbox = DCMotor.CIM(2)

    def __init__(self) -> None:
        super().__init__()
        Logger.recordMetadata("Robot", "Romi")

        print(f"isRealRomiMode()={constants.isRealRomiMode()} RobotBase.isReal()={RobotBase.isReal()} constants.kRobotMode={constants.kRobotMode}")

        match constants.kRobotMode:
            case constants.RobotModes.REAL|constants.RobotModes.SIMULATION:
                deploy_config = wpilib.deployinfo.getDeployData()
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
            "Auto Routine 1", AutoDriveForward1Sec(self, 1)
        )
        self.chooser.addOption("Auto Routine 2 - same as 1", AutoDriveForward1Sec(self, 2))
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

        if constants.kRobotMode == constants.RobotModes.SIMULATION:
            self.sim = DifferentialDrivetrainSim.createKitbotSim(
                self.kGearbox,
                self.kMotorReduction,
                self.kWheelRadiusM,
            )
        else:
            self.sim = None

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            (pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            (pi * self.kWheelDiameterInch) / self.kCountsPerRevolution
        )

        self.kinematics = DifferentialDriveKinematics(self.kTrackWidthM)

        self.updateInputsEncodersToDistances()

        self._lastLeftEncoderDistanceM = self._leftEncoderDistanceM
        self._lastRightEncoderDistanceM = self._rightEncoderDistanceM

        self._rawOdometryRotation = Rotation2d()

        self.poseEstimator = DifferentialDrivePoseEstimator(
            self.kinematics, Rotation2d(), 0.0, 0.0, Pose2d()
        )

        self.zeroGamePadInputs()
        self.updateInputs()



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
        
        # Run Robot Casserole's Signal Wrangler to publish logs under their system
        SignalWrangler().publishPeriodic()



    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.zeroGamePadInputs()
        self.autonomousCommand = self.chooser.getSelected()

        if self.autonomousCommand is not None:
            self.autonomousCommand.start()
            CommandScheduler.getInstance().schedule(self.autonomousCommand.getCommand())

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
       # TODO was self.autonomousCommand.periodic()
        self.updateInputs()
        self.updateMotorSpeeds()


    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        pass

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        self.updateGamePadInputs()
        self.updateInputs()
        self.updateMotorSpeeds()

    def testInit(self) -> None:
        pass

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def updateInputsEncodersToDistances(self):
        if self.sim is not None:
            self._leftDriveDistanceInches = self.sim.getLeftPositionInches()
            self._rightDriveDistanceInches = self.sim.getRightPositionInches()
        else:
            self._leftDriveDistanceInches = -self.leftEncoder.getDistance()
            self._rightDriveDistanceInches = -self.rightEncoder.getDistance()

        self._leftEncoderDistanceM = self._leftDriveDistanceInches* self.kMetersPerInch
        self._rightEncoderDistanceM = self._rightDriveDistanceInches * self.kMetersPerInch

    def zeroGamePadInputs(self):
        self._rawForwardCommand = 0.0
        self._rawRotationCommand = 0.0
        self._slowMultiplier = 0.25
        self._forwardCommand = 0.0
        self._rotationCommand = 0.0

    def updateGamePadInputs(self):
        self._rawForwardCommand = -self.controller.getRawAxis(1)
        self._rawRotationCommand = -self.controller.getRawAxis(4)
        self._slowMultiplier = 1.0 if (self.controller.getRawButton(6)) else 0.25

        self.setForwardAndRotationCommands(
            forwardCommand=deadband(self._rawForwardCommand, 0.1) * self.slowMultiplier(),
            rotationCommand=deadband(self._rawRotationCommand, 0.1) * self.slowMultiplier()
        )

    def updateInputs(self):


        self.updateInputsEncodersToDistances()

        self._yawPositionDeg = -self.gyro.getAngle() / self.kRadiansPerDegree
        self._yawVelocityDegPerSec = -self.gyro.getRate() / self.kRadiansPerDegree
        self._yawPosition = Rotation2d.fromDegrees(self._yawPositionDeg)

        twist = self.kinematics.toTwist2d(
            self._leftEncoderDistanceM - self._lastLeftEncoderDistanceM,
            self._rightEncoderDistanceM - self._lastRightEncoderDistanceM,
        )

        # TODO This seems misleading it's called rawGyroRotation,
        #  but it seems entirely based upon wheel odometry
        self._rawOdometryRotation = self._rawOdometryRotation + Rotation2d(twist.dtheta)

        self._lastLeftEncoderDistanceM = self._leftEncoderDistanceM
        self._lastRightEncoderDistanceM = self._rightEncoderDistanceM

        self.poseEstimator.update(
            self._rawOdometryRotation, self._leftEncoderDistanceM, self._rightEncoderDistanceM
        )

    def setForwardAndRotationCommands(self, forwardCommand: float, rotationCommand: float):
        self._forwardCommand = forwardCommand
        self._rotationCommand = rotationCommand

    def updateMotorSpeeds(self):
        speeds = DifferentialDrive.arcadeDriveIK(self._forwardCommand, self._rotationCommand, False)
        kRatioToVolts = 12.0
        self._leftMotorSetVoltage = speeds.left * kRatioToVolts
        self._rightMotorSetVoltage = speeds.right * kRatioToVolts
        self.leftMotor.setVoltage(self._leftMotorSetVoltage)
        self.rightMotor.setVoltage(self._rightMotorSetVoltage)
        if self.sim is not None:
            self.sim.setInputs(
                clamp(self._leftMotorSetVoltage, -12.0, 12.0),
                clamp(self._rightMotorSetVoltage, -12.0, 12.0),
            )
            self.sim.update(constants.kRobotPeriod)

    @autolog_output(key="Inputs/autonomousCommandNum", custom_type="int")
    def getAutonomousCommandNum(self) -> int:
        result = -1
        if self.autonomousCommand is not None:
            result = self.autonomousCommand.option_number
        return result

    @autolog_output(key="Inputs/rawForwardCommand", custom_type="ratio")
    def rawForwardCommand (self) -> float:
        return self._rawForwardCommand

    @autolog_output(key="Inputs/rawRotationCommand", custom_type="ratio")
    def rawRotationCommand (self) -> float:
        return self._rawRotationCommand

    @autolog_output(key="Inputs/slowMultiplier", custom_type="ratio")
    def slowMultiplier(self) -> float:
        return self._slowMultiplier

    @autolog_output(key="Inputs/forwardCommand", custom_type="ratio")
    def forwardCommand(self) -> float:
        return self._forwardCommand

    @autolog_output(key="Inputs/rotationCommand", custom_type="ratio")
    def rotationCommand(self) -> float:
        return self._rotationCommand

    @autolog_output(key="Inputs/leftMotorVolts", custom_type="V")
    def leftMotorVoltage(self) -> float:
        return self.leftMotor.getVoltage()

    @autolog_output(key="Inputs/rightMotorVolts", custom_type="V")
    def rightMotorVoltage(self) -> float:
        return self.leftMotor.getVoltage()

    @autolog_output(key="Inputs/leftDriveDistanceInches", custom_type="in")
    def leftDriveDistanceInches(self) -> float:
        return -self.leftEncoder.getDistance()

    @autolog_output(key="Inputs/rightDriveDistanceInches", custom_type="in")
    def rightDriveDistanceInches(self) -> float:
        return -self.rightEncoder.getDistance()

    @autolog_output(key="Inputs/yawPositionDeg", custom_type="deg")
    def yawPositionDeg(self) -> float:
        return self._yawPositionDeg

    @autolog_output(key="Inputs/yawVelocityDegPerSec", custom_type="deg/s")
    def yawVelocityDegPerSec(self) -> float:
        return self._yawVelocityDegPerSec

    @autolog_output(key="Inputs/yawPosition")
    def yawPosition(self) -> Rotation2d:
        return self._yawPosition

    @autolog_output(key="Outputs/rawOdometryRotation")
    def rawOdometryRotation(self) -> Rotation2d:
        return self._rawOdometryRotation

    @autolog_output(key="Odometry/Robot")
    def getPose(self) -> Pose2d:
        return self.poseEstimator.getEstimatedPosition()


if __name__ == "__main__":
    wpilib.run(MyRobot)
