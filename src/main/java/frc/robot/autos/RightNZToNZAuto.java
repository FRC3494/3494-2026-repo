package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class RightNZToNZAuto extends AutoBase {
  public String getName() {
    return "RightNZToNZ";
  }

  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.RightTrenchStartingPosition;
  }

  public AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory rightTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory rightMiddleNZToShoot =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightMiddleNZToShoot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightMiddleNZToShoot_RED.asAutoTraj(routine);
    AutoTrajectory rightShootToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightShootToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightShootToNZ_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("1"),
                rightTrenchToNZ.resetOdometry(),
                print("2"),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    rightTrenchToNZ.cmd()),
                print("3")));

    rightTrenchToNZ.atTime("NZIntake").onTrue(robotCommands.intake());

    rightTrenchToNZ.done().onTrue(sequence(robotCommands.stopIntake(), rightMiddleNZToShoot.cmd()));

    rightMiddleNZToShoot.atTime("StartFlywheel").onTrue(robotCommands.startFlywheel());

    rightMiddleNZToShoot
        .done()
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(7),
                robotCommands.stopShootNoDelay(),
                robotCommands.startIntake(),
                rightShootToNZ.cmd()));

    return routine;
  }
}
