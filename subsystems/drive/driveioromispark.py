#TODO from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from dataclasses import dataclass

from pykit.logger import Logger

from subsystems.drive.driveio import DriveIO
from subsystems.drive import driveconstants
from wpilib import Spark, Encoder, XboxController
from wpimath.controller import PIDController



from constants.math import kRadiansPerRevolution, kSecondsPerMinute
from util.helpfulmath import deadband
from util.sparkutil import tryUntilOk, isOk, isOkMulti
from math import pi
from subsystems.drive.driveconstants import kLeftInverted, kRightInverted, kWheelDiameterInch, kCountsPerRevolution, \
    kWheelRadiusM, kWheelRadiusInch


class DriveIORomiSpark(DriveIO):

    def __init__(self) -> None:
        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftLeader = Spark(0)
        self.leftLeader.setInverted(kLeftInverted)
        self.rightLeader = Spark(1)
        self.rightLeader.setInverted(kRightInverted)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = Encoder(4, 5)
        self.rightEncoder = Encoder(6, 7)

        self.resetEncoders()

        # Use revolutions as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            1.0 / kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            1.0 / kCountsPerRevolution
        )
        print(f"kWheelDiameterInch={kWheelDiameterInch} kCountsPerRevolution={kCountsPerRevolution} result={(pi * kWheelDiameterInch) / kCountsPerRevolution}")

        self.leftPID = PIDController(driveconstants.kRealKp, 0.0, driveconstants.kRealKd)
        self.rightPID = PIDController(driveconstants.kRealKp, 0.0, driveconstants.kRealKd)

        self.debugController: XboxController|None = None

    def updateInputs(self, inputs: DriveIO.DriveIOInputs) -> None:

        inputs.leftPositionCount = self.leftEncoder.get()
        inputs.rightPositionCount = self.rightEncoder.get()
        inputs.leftDriveDistanceInches = self.leftEncoder.getDistance() * pi * kWheelDiameterInch
        inputs.rightDriveDistanceInches = self.rightEncoder.getDistance() * pi * kWheelDiameterInch
        inputs.leftPositionRad = inputs.leftDriveDistanceInches / kWheelRadiusInch
        inputs.rightPositionRad = inputs.rightDriveDistanceInches / kWheelRadiusInch
        inputs.leftVelocityRadPerSec = deadband(self.leftEncoder.getRate() * 2.0 * pi, 1e-10)
        inputs.rightVelocityRadPerSec = deadband(self.rightEncoder.getRate() * 2.0 * pi, 1e-10)
        inputs.leftAppliedVolts = self.leftLeader.getVoltage()
        inputs.rightAppliedVolts = self.rightLeader.getVoltage()

    def setVoltage(self, leftVolts: float, rightVolts: float) -> None:
        _leftVolts = float(leftVolts)
        _rightVolts = float(rightVolts)
        self.leftSetVolts = _leftVolts
        self.rightSetVolts = _rightVolts
        self.leftLeader.setVoltage(_leftVolts)
        self.rightLeader.setVoltage(_rightVolts)

    def setVelocity(
        self,
        inputs: DriveIO.DriveIOInputs,
        leftRadPerSec: float,
        rightRadPerSec: float,
        leftFFVolts: float,
        rightFFVolts: float,
    ) -> None:
        self.leftPID.setSetpoint(leftRadPerSec)
        self.rightPID.setSetpoint(rightRadPerSec)
        leftPIDVolts = self.leftPID.calculate(inputs.leftVelocityRadPerSec)
        rightPIDVolts = self.rightPID.calculate(inputs.rightVelocityRadPerSec)

        Logger.recordOutput("Drive/leftPIDVolts", leftPIDVolts)
        Logger.recordOutput("Drive/rightPIDVolts", rightPIDVolts)
        Logger.recordOutput("Drive/leftRadPerSecCmd", leftRadPerSec)
        Logger.recordOutput("Drive/rightRadPerSecCmd", rightRadPerSec)

        if self.debugController is not None and self.debugController.getAButton() and not self.debugController.getBButton():
            leftV = leftPIDVolts
            rightV = rightPIDVolts
        elif self.debugController is not None and not self.debugController.getAButton() and self.debugController.getBButton():
            leftV = leftFFVolts
            rightV = rightFFVolts
        else:
            leftV = leftPIDVolts + leftFFVolts
            rightV = rightPIDVolts + rightFFVolts

        self.setVoltage(leftV, rightV)



    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.leftEncoder.reset()
        self.rightEncoder.reset()