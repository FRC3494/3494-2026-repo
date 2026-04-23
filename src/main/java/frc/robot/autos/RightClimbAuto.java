package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import choreo.auto.AutoChooser;
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
import java.util.Map;

public class RightClimbAuto {
  public static final String name = "RightClimb";
  public static final Pose2d startingPose = ChoreoVars.Poses.RightTrenchStartingPosition;

  public static AutoRoutine getRoutine(
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

  public static void loadAuto(
      Map<String, Pose2d> startingPoseMap,
      AutoChooser autoChooser,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    autoChooser.addRoutine(
        name + "_BLUE",
        () ->
            getRoutine(
                name + "_BLUE", Alliance.Blue, autoFactory, robotCommands, drive, shooterAimModel));
    autoChooser.addRoutine(
        name + "_RED",
        () ->
            getRoutine(
                name + "_RED", Alliance.Red, autoFactory, robotCommands, drive, shooterAimModel));

    startingPoseMap.put(name + "_BLUE", startingPose);
    startingPoseMap.put(name + "_RED", startingPose);
  }
}
