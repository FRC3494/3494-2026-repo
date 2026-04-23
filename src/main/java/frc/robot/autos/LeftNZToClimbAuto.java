package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;
import java.util.HashMap;

public class LeftNZToClimbAuto {
  public static final String name = "LeftNZToClimb";
  public static final Pose2d startingPose = ChoreoVars.Poses.LeftTrenchStartingPosition;

  public static AutoRoutine getRoutine(
      String routineName,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(routineName);

    AutoTrajectory leftTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory middleNZToLeftClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.MiddleNZToLeftClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.MiddleNZToLeftClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftTrenchToNZ.resetOdometry(),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    leftTrenchToNZ.cmd())));

    leftTrenchToNZ.atTime("NZIntake").onTrue(robotCommands.intake().andThen(print("NZIntake")));

    leftTrenchToNZ.done().onTrue(sequence(robotCommands.stopIntake(), middleNZToLeftClimb.cmd()));

    // middleNZToLeftClimb
    //     .atPose("ClimberUp", Units.inchesToMeters(12), Math.PI)
    //     .onTrue(
    //         robotCommands
    //             .startClimberUp()
    //             .andThen(
    //                 print(
    //                     "ClimberUp")));

    middleNZToLeftClimb
        .atTime("StartFlywheel")
        .onTrue(robotCommands.startFlywheel().andThen(print("StartFlywheel")));

    middleNZToLeftClimb
        .done()
        .onTrue(
            sequence(
                    robotCommands.runClimberUp().deadlineFor(robotCommands.shoot()),
                    parallel(
                        sequence(
                            AutoAlignCommand.alignSequence(
                                drive,
                                alliance == Alliance.Blue
                                    ? climbSetupPoseDepot_BLUE
                                    : climbSetupPoseDepot_RED,
                                alliance == Alliance.Blue
                                    ? climbPoseDepot_BLUE
                                    : climbPoseDepot_RED),
                            robotCommands.creepBackward()),
                        sequence(
                            waitUntil(() -> Timer.getMatchTime() <= 5)
                                .deadlineFor(robotCommands.shoot()),
                            runOnce(
                                () -> {
                                  shooterAimModel.setTurretTrim(
                                      turretTrimDefaultRot + Units.degreesToRotations(-5.0));
                                },
                                shooterAimModel),
                            parallel(
                                robotCommands.runClimberMidWithCurrent(),
                                robotCommands.runIntakeUp()),
                            runOnce(
                                () -> {
                                  shooterAimModel.setTurretTrim(
                                      turretTrimDefaultRot + Units.degreesToRotations(-10.0));
                                },
                                shooterAimModel),
                            robotCommands.shoot())))
                .finallyDo(
                    () -> {
                      shooterAimModel.setTurretTrim(turretTrimDefaultRot);
                    }));

    return routine;
  }

  public static void loadAuto(
      HashMap<String, Pose2d> startingPoseMap,
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
