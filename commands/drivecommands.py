from typing import Callable
from commands2 import Command, cmd
from pykit.logger import Logger
from wpilib import Timer
from wpilib.drive import DifferentialDrive

from subsystems.drive.drive import Drive
from util.helpfulmath import deadband

from subsystems.drive import driveconstants


class DriveCommands:
    deadband: float = 0.001 # TODO put me back to 0.1
    ff_ramp_rate: float = driveconstants.kFfRampRateForTestCharacterization  # volts / sec

    @staticmethod
    def arcadeDriveOpenLoop(
        drive: Drive,
        forward: Callable[[], float],
        rotation: Callable[[], float],
        slowMultiplier: Callable[[], float],
    ) -> Command:
        def run():
            fwd = deadband(forward(), DriveCommands.deadband) * slowMultiplier()
            rot = deadband(rotation(), DriveCommands.deadband) * slowMultiplier()

            bothSideOfChassisForwardSpeeds = DifferentialDrive.arcadeDriveIK(fwd, rot, False)
            drive.runOpenLoop(
                bothSideOfChassisForwardSpeeds.left * 12.0,
                bothSideOfChassisForwardSpeeds.right * 12.0,
            ) # TODO make 12.0 DRY.

        return cmd.run(run, drive).withName("Arcade Drive Open Loop")

    @staticmethod
    def arcadeDriveClosedLoop(
        drive: Drive,
        forward: Callable[[], float],
        rotation: Callable[[], float],
        slowMultiplier: Callable[[], float],
        debugRunOpenLoopVolts: Callable[[], float]

    ) -> Command:
        def run():
            fwd = deadband(forward(), DriveCommands.deadband) * slowMultiplier()
            rot = deadband(rotation(), DriveCommands.deadband) * slowMultiplier()
            bothSideOfChassisForwardSpeeds = DifferentialDrive.arcadeDriveIK(fwd, rot, False)

            if debugRunOpenLoopVolts():
                drive.runOpenLoop(
                    bothSideOfChassisForwardSpeeds.left*12.0,
                    bothSideOfChassisForwardSpeeds.right*12.0
                ) # TODO make 12.0 DRY.
            else:
                drive.runClosedLoopParameters(
                    bothSideOfChassisForwardSpeeds.left * driveconstants.kMaxSpeedMetersPerSecond,
                    bothSideOfChassisForwardSpeeds.right * driveconstants.kMaxSpeedMetersPerSecond,
                )

        return cmd.run(run, drive).withName("Arcade Drive Closed Loop")

    @staticmethod
    def arcadeDriveClosedLoopSlewRateLimited(
        drive: Drive,
        forward: Callable[[], float],
        rotation: Callable[[], float],
        slowMultiplier: Callable[[], float],
        debugRunOpenLoopVolts: Callable[[], float]

    ) -> Command:
        def setup():
            drive.oldForward = 0.0
            drive.oldRotation = 0.0
            drive.maxAlwChangeFwd = 0.6 # tested (carpet): no slipping from still, some slipping fwd to backward
            # drive.maxAlwChangeFwd = 0.08 # tested (carpet): almost no slipping at all, 0.5 sec from full fwd to full backward
            drive.maxAlwChangeRot = 2.0 # set at max (so it won't limit) because rotating didn't slip at all on carpet

        def run():
            fwd = deadband(forward(), DriveCommands.deadband) * slowMultiplier()
            rot = deadband(rotation(), DriveCommands.deadband) * slowMultiplier()

            difFwd = fwd - drive.oldForward
            if abs(difFwd) > drive.maxAlwChangeFwd:
                if difFwd > 0.0:
                    cmdForward = drive.oldForward + drive.maxAlwChangeFwd
                else:
                    cmdForward = drive.oldForward - drive.maxAlwChangeFwd
            else:
                cmdForward = fwd
            # This code works as a slew rate limiter for the right joystick
            difRot = rot - drive.oldRotation
            if abs(difRot) > drive.maxAlwChangeRot:
                if difRot > 0.0:
                    cmdRot = drive.oldRotation + drive.maxAlwChangeRot
                else:
                    cmdRot = drive.oldRotation - drive.maxAlwChangeRot
            else:
                cmdRot = rot
            # Variables to remember the past value
            drive.oldForward = cmdForward
            drive.oldRotation = cmdRot
            # It only prints this value once
            print(f"cmdForward = {cmdForward}")
            print(f"cmdRot = {cmdRot}")
            # These logs populate but do not change
            Logger.recordOutput("Drive/leftYRateLimited", cmdForward)
            Logger.recordOutput("Drive/rightXRateLimited", cmdRot)


            bothSideOfChassisForwardSpeeds = DifferentialDrive.arcadeDriveIK(cmdForward, cmdRot, False)

            if debugRunOpenLoopVolts():
                drive.runOpenLoop(
                    bothSideOfChassisForwardSpeeds.left*12.0,
                    bothSideOfChassisForwardSpeeds.right*12.0
                ) # TODO make 12.0 DRY.
            else:
                drive.runClosedLoopParameters(
                    bothSideOfChassisForwardSpeeds.left * driveconstants.kMaxSpeedMetersPerSecond,
                    bothSideOfChassisForwardSpeeds.right * driveconstants.kMaxSpeedMetersPerSecond,
                )

        return cmd.runOnce(setup).andThen(cmd.run(run, drive).withName("Arcade Drive Closed Loop"))

    @staticmethod
    def feedForwardCharacterization(drive: Drive) -> Command:
        velocitySamples: list[float] = []
        voltageSamples: list[float] = []
        timer = Timer()

        def setup():
            velocitySamples.clear()
            voltageSamples.clear()
            timer.restart()

        def run():
            voltage = timer.get() * DriveCommands.ff_ramp_rate
            drive.runOpenLoop(voltage, voltage)
            velocitySamples.append(drive.getCharacterizationVelocityRadPerS())
            voltageSamples.append(voltage)

        def end(_interrupted: bool):
            n = len(velocitySamples)
            sumX = sum(velocitySamples)
            sumY = sum(voltageSamples)
            sumXY = sum(velocitySamples[i] * voltageSamples[i] for i in range(n))
            sumX2 = sum(v**2 for v in velocitySamples)
            kS = 0.0
            kV = 0.0
            divisor = (n * sumX2 - sumX * sumX)
            if divisor != 0.0:
                kS = (sumY * sumX2 - sumX * sumXY) / divisor
                kV = (n * sumXY - sumX * sumY) / divisor

            print("************************************************************")
            print(f"Feed Forward Characterization Results: \nkS = {kS}\nkV = {kV}")

        return (
            cmd.runOnce(setup)
            .andThen(cmd.run(run, drive).withName("FeedForwardCharacterization"))
            .finallyDo(end)
        )
