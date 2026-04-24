package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class RightClimbAuto extends AutoBase {
  @Override
  public String getName() {
    return "RightClimb";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.RightTrenchStartingPosition;
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

    AutoTrajectory rightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightTrenchToClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightTrenchToClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                rightClimb.resetOdometry(),
                robotCommands.enableAutoShooterSettings(),
                robotCommands.enableAutoTurret(),
                waitSeconds(0.5),
                robotCommands.runClimberUp(),
                robotCommands.shoot().withTimeout(3),
                parallel(rightClimb.cmd(), robotCommands.spinDownFromShoot())));

    rightClimb
        .done()
        .onTrue(
            parallel(
                sequence(
                    new AutoAlignCommand(
                        alliance == Alliance.Blue ? climbPoseOutpost_BLUE : climbPoseOutpost_RED,
                        drive),
                    robotCommands.creepBackward()),
                sequence(waitSeconds(1), robotCommands.runClimberMid())));

    return routine;
  }
}
