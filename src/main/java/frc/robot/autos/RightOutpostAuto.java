package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;

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

public class RightOutpostAuto extends AutoBase {
  public String getName() {
    return "RightOutpost";
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
    AutoTrajectory rightMiddleNZToOutpost_0 =
        ChoreoTraj.RightMiddleNZToOutpost.segment(0).asAutoTraj(routine);
    AutoTrajectory rightMiddleNZToOutpost_1 =
        ChoreoTraj.RightMiddleNZToOutpost.segment(1).asAutoTraj(routine);
    AutoTrajectory outpostToRightClimb = ChoreoTraj.OutpostToRightClimb.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                rightTrenchToNZ.resetOdometry(),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    rightTrenchToNZ.cmd())));

    rightTrenchToNZ.atTime("NZIntake").onTrue(robotCommands.intake());

    rightTrenchToNZ
        .done()
        .onTrue(sequence(robotCommands.stopIntake(), rightMiddleNZToOutpost_0.cmd()));

    rightMiddleNZToOutpost_0.atTime("StartFlywheel").onTrue(robotCommands.startFlywheel());

    rightMiddleNZToOutpost_0
        .done()
        .onTrue(rightMiddleNZToOutpost_1.cmd().deadlineFor(robotCommands.shoot()));

    rightMiddleNZToOutpost_1
        .done()
        .onTrue(
            sequence(
                robotCommands.runClimberUp().deadlineFor(robotCommands.shoot()),
                outpostToRightClimb.cmd().deadlineFor(robotCommands.shoot())));

    outpostToRightClimb
        .done()
        .onTrue(
            parallel(
                    sequence(
                        AutoAlignCommand.alignSequence(
                            drive,
                            alliance == Alliance.Blue
                                ? climbSetupPoseOutpost_BLUE
                                : climbSetupPoseOutpost_RED,
                            alliance == Alliance.Blue
                                ? climbPoseOutpost_BLUE
                                : climbPoseOutpost_RED),
                        parallel(
                            robotCommands.creepBackward(),
                            sequence(
                                waitUntil(() -> Timer.getMatchTime() <= 4)
                                    .deadlineFor(robotCommands.shoot()),
                                runOnce(
                                    () -> {
                                      shooterAimModel.setTurretTrim(
                                          turretTrimDefaultRot + Units.degreesToRotations(5.0));
                                    },
                                    shooterAimModel),
                                parallel(
                                    robotCommands.runClimberMidWithCurrent(),
                                    robotCommands.runIntakeUp()),
                                runOnce(
                                    () -> {
                                      shooterAimModel.setTurretTrim(
                                          turretTrimDefaultRot + Units.degreesToRotations(10.0));
                                    },
                                    shooterAimModel),
                                robotCommands.shoot()))))
                .finallyDo(
                    () -> {
                      shooterAimModel.setTurretTrim(turretTrimDefaultRot);
                    }));

    return routine;
  }
}
