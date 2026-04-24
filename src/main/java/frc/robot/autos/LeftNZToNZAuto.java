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

public class LeftNZToNZAuto extends AutoBase {
  @Override
  public String getName() {
    return "LeftNZToNZ";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftTrenchStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory leftTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory leftMiddleNZToShoot =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftMiddleNZToShoot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftMiddleNZToShoot_RED.asAutoTraj(routine);
    AutoTrajectory leftShootToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftShootToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftShootToNZ_RED.asAutoTraj(routine);

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

    leftTrenchToNZ.atTime("NZIntake").onTrue(robotCommands.intake());

    leftTrenchToNZ.done().onTrue(sequence(robotCommands.stopIntake(), leftMiddleNZToShoot.cmd()));

    leftMiddleNZToShoot.atTime("StartFlywheel").onTrue(robotCommands.startFlywheel());

    leftMiddleNZToShoot
        .done()
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(7),
                robotCommands.stopShootNoDelay(),
                robotCommands.startIntake(),
                leftShootToNZ.cmd()));

    return routine;
  }
}
