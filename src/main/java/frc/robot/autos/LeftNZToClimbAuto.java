package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.climbPoseDepot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.util.choreo.ChoreoTraj;

public class LeftNZToClimbAuto {
  public static AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory leftTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory middleNZToLeftClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.MiddleNZToLeftClimb.asAutoTraj(routine)
            : ChoreoTraj.MiddleNZToLeftClimb.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("1"),
                leftTrenchToNZ.resetOdometry(),
                print("2"),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    leftTrenchToNZ.cmd()),
                print("3")));

    leftTrenchToNZ.atTime("LeftNZIntake").onTrue(robotCommands.intake());

    leftTrenchToNZ.done().onTrue(sequence(robotCommands.stopIntake(), middleNZToLeftClimb.cmd()));

    middleNZToLeftClimb.atTime("LeftClimbShoot").onTrue(robotCommands.shoot());

    middleNZToLeftClimb
        .done()
        .onTrue(
            sequence(
                parallel(
                    sequence(
                        new AutoAlignCommand(climbPoseDepot, drive), robotCommands.creepBackward()),
                    sequence(waitSeconds(1), robotCommands.climberMid()))));

    return routine;
  }
}
