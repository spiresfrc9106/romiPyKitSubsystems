from typing import Optional
from commands2 import Subsystem
from pathplannerlib.commands import (
    DriveFeedforwards,
    PPLTVController,
    PathPlannerLogging,
)
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib import DriverStation
from commands2.sysid import SysIdRoutine
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics,
    DifferentialDriveWheelSpeeds,
)
from wpimath.estimator import DifferentialDrivePoseEstimator

from pathplannerlib.auto import AutoBuilder

from subsystems.drive.driveio import DriveIO
from subsystems.drive.gyroio import GyroIO
from subsystems.drive import driveconstants

import constants
from util.helpfulmath import sign
from util.sysidlog import sysIdStateToStr


@autologgable_output
class Drive(Subsystem):
    def __init__(self, io: DriveIO, gyroIO: GyroIO) -> None:
        self.leftOldRadPerS = 0
        self.rightOldRadPerS = 0
        self.io = io
        self.gyroIO = gyroIO

        self.inputs = DriveIO.DriveIOInputs()
        self.gyroInputs = GyroIO.GyroIOInputs()

        self.kinematics = DifferentialDriveKinematics(driveconstants.kTrackWidthM)

        self.kS = (
            driveconstants.kSimKs
            if constants.kRobotMode == constants.RobotModes.SIMULATION
            else driveconstants.kRealKs
        )
        self.kV = (
            driveconstants.kSimKv
            if constants.kRobotMode == constants.RobotModes.SIMULATION
            else driveconstants.kRealKv
        )
        self.kA = (
            driveconstants.kSimKa
            if constants.kRobotMode == constants.RobotModes.SIMULATION
            else driveconstants.kRealKa
        )

        self.poseEstimator = DifferentialDrivePoseEstimator(
            self.kinematics, Rotation2d(), 0.0, 0.0, Pose2d()
        )
        self.rawOdometryRotation = Rotation2d()
        self.lastLeftDriveDistanceM = 0.0
        self.lastRightDriveDistanceM = 0.0

        AutoBuilder.configure(
            self.getPose,
            self.setPose,
            self.getChassisSpeeds,
            self.runClosedLoop,
            PPLTVController(
                constants.kRobotPeriod, driveconstants.kMaxSpeedMetersPerSecond
            ),
            driveconstants.kPPConfig,
            (lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed),
            self,
        )

        PathPlannerLogging.setLogActivePathCallback(
            lambda activePath: Logger.recordOutput("Odometry/Trajectory", activePath)
        )
        PathPlannerLogging.setLogTargetPoseCallback(
            lambda targetPose: Logger.recordOutput(
                "Odometry/TrajectorySetpoint", targetPose
            )
        )


        self.sysid = SysIdRoutine(
            SysIdRoutine.Config(
                1.0, # was 1
                7.0,
                10.0, # was 10
                lambda state: Logger.recordOutput(
                    "Drive/SysIdState", sysIdStateToStr(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                (lambda volts: self.runOpenLoop(volts, volts)), (lambda x: None), self
            ),
        )

    def periodic(self) -> None:
        self.io.updateInputs(self.inputs)
        self.gyroIO.updateInputs(self.gyroInputs)
        Logger.processInputs("Drive", self.inputs)
        Logger.processInputs("Drive/Gyro", self.gyroInputs)

        if self.gyroInputs.connected:
            self.rawOdometryRotation = self.gyroInputs.yawPosition
        else:
            twistFromOdometry = self.kinematics.toTwist2d(
                self.getLeftDriveDistanceM() - self.lastLeftDriveDistanceM,
                self.getRightDriveDistanceM() - self.lastRightDriveDistanceM,
            )
            self.rawOdometryRotation = self.rawOdometryRotation + Rotation2d(twistFromOdometry.dtheta)

        self.lastLeftDriveDistanceM = self.getLeftDriveDistanceM()
        self.lastRightDriveDistanceM = self.getRightDriveDistanceM()

        self.poseEstimator.update(
            self.rawOdometryRotation, self.getLeftDriveDistanceM(), self.getRightDriveDistanceM()
        )

    def runClosedLoop(
        self, chassisSpeedsMPS: ChassisSpeeds, _feedForwards: Optional[DriveFeedforwards] = None
    ):
        wheelSpeedsMPS = self.kinematics.toWheelSpeeds(chassisSpeedsMPS)
        self.runClosedLoopParameters(wheelSpeedsMPS.left, wheelSpeedsMPS.right)

    def runClosedLoopParameters(self, leftSpeedMPS: float, rightSpeedMPS: float):
        leftRadPerS = leftSpeedMPS / driveconstants.kWheelRadiusM
        rightRadPerS = rightSpeedMPS / driveconstants.kWheelRadiusM
        leftAccRadPerS2 = (leftRadPerS - self.leftOldRadPerS) / 0.02
        rightAccRadPerS2 = (rightRadPerS - self.rightOldRadPerS) / 0.02

        Logger.recordOutput("Drive/leftSetpointMPS", leftSpeedMPS)
        Logger.recordOutput("Drive/rightSetpointMPS", leftSpeedMPS)

        Logger.recordOutput("Drive/leftSetpointRPS", leftRadPerS)
        Logger.recordOutput("Drive/rightSetpointRPS", rightRadPerS)

        leftFF = self.kS * sign(leftRadPerS) + self.kV * leftRadPerS + self.kA * leftAccRadPerS2
        rightFF = self.kS * sign(rightRadPerS) + self.kV * rightRadPerS + self.kA * rightAccRadPerS2

        Logger.recordOutput("Drive/leftFFVolts", leftFF)
        Logger.recordOutput("Drive/rightFFVolts", rightFF)

        self.io.setVelocity(self.inputs, leftRadPerS, rightRadPerS, leftFF, rightFF)

        self.leftOldRadPerS = leftRadPerS
        self.rightOldRadPerS = rightRadPerS

    def runOpenLoop(self, leftV: float, rightV: float) -> None:
        self.io.setVoltage(leftV, rightV)

    def stop(self):
        self.runOpenLoop(0, 0)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction):
        return self.sysid.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction):
        return self.sysid.dynamic(direction)

    @autolog_output(key="Odometry/Robot")
    def getPose(self) -> Pose2d:
        return self.poseEstimator.getEstimatedPosition()

    def getRotation(self) -> Rotation2d:
        return self.getPose().rotation()

    def setPose(self, pose: Pose2d) -> None:
        self.poseEstimator.resetPosition(
            self.rawOdometryRotation, self.getLeftDriveDistanceM(), self.getRightDriveDistanceM(), pose
        )

    def addVisionMeasurement(self, visionPose: Pose2d, timestamp: float):
        self.poseEstimator.addVisionMeasurement(visionPose, timestamp)

    @autolog_output(key="Drive/leftDriveDistanceM")
    def getLeftDriveDistanceM(self) -> float:
        return self.inputs.leftPositionRad * driveconstants.kWheelRadiusM

    @autolog_output(key="Drive/rightDriveDistanceM")
    def getRightDriveDistanceM(self) -> float:
        return self.inputs.rightPositionRad * driveconstants.kWheelRadiusM

    @autolog_output(key="Drive/leftDriveVelocityMPS")
    def getLeftDriveVelocityMPS(self) -> float:
        return self.inputs.leftVelocityRadPerSec * driveconstants.kWheelRadiusM

    @autolog_output(key="Drive/leftDriveVelocityIPS")
    def getLeftDriveVelocityIPS(self) -> float:
        return self.inputs.leftVelocityRadPerSec * driveconstants.kWheelRadiusInch

    @autolog_output(key="Drive/rightDriveVelocityMPS")
    def getRightDriveVelocityMPS(self) -> float:
        return self.inputs.rightVelocityRadPerSec * driveconstants.kWheelRadiusM

    @autolog_output(key="Drive/leftDriveVelocityIPS")
    def getRightDriveVelocityIPS(self) -> float:
        return self.inputs.leftVelocityRadPerSec * driveconstants.kWheelRadiusInch

    def getCharacterizationVelocityRadPerS(self) -> float:
        return (
            self.inputs.leftVelocityRadPerSec + self.inputs.rightVelocityRadPerSec
        ) / 2.0

    def getChassisSpeeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(
                driveconstants.kWheelRadiusM * self.getLeftDriveVelocityMPS(),
                driveconstants.kWheelRadiusM * self.getRightDriveVelocityMPS(),
            )
        )
