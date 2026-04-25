package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.Constants.alliance;

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

public class LeftHubToClimbAuto extends AutoBase {
  @Override
  public String getName() {
    return "LeftHubToClimb";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftHubStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(routineName);

    AutoTrajectory leftHubToClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftHubToClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftHubToClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftHubToClimb.resetOdometry(),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    leftHubToClimb.cmd())));

    leftHubToClimb
        .done()
        .onTrue(
            sequence(
                robotCommands.runClimberUp().deadlineFor(robotCommands.shoot()),
                robotCommands.shoot().withTimeout(2),
                Autos.climbDepot(robotCommands, drive, shooterAimModel)));

    return routine;
  }
}
