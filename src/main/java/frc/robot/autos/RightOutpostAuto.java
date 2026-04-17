package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.util.choreo.ChoreoTraj;

public class RightOutpostAuto {
  public static AutoRoutine getRoutine(
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
    AutoTrajectory rightMiddleNZToOutpost = ChoreoTraj.RightMiddleNZToOutpost.asAutoTraj(routine);
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
        .onTrue(sequence(robotCommands.stopIntake(), rightMiddleNZToOutpost.cmd()));

    rightMiddleNZToOutpost.atTime("StartFlywheel").onTrue(robotCommands.startFlywheel());

    rightMiddleNZToOutpost.atTime("Shoot").onTrue(robotCommands.shoot());

    rightMiddleNZToOutpost
        .done()
        .onTrue(
            sequence(
                robotCommands.runClimberUp().deadlineFor(robotCommands.shoot()),
                outpostToRightClimb.cmd().deadlineFor(robotCommands.shoot())));

    outpostToRightClimb
        .done()
        .onTrue(
            parallel(
                    robotCommands.shoot(),
                    sequence(
                        AutoAlignCommand.alignSequence(
                            drive, climbSetupPoseOutpost, climbPoseOutpost),
                        parallel(
                            robotCommands.creepBackward(),
                            sequence(
                                waitUntil(() -> Timer.getMatchTime() <= 4),
                                runOnce(
                                    () -> {
                                      shooterAimModel.setTurretTrim(
                                          turretTrimDefaultRot + Units.degreesToRotations(5.0));
                                    },
                                    shooterAimModel),
                                robotCommands.runClimberMidWithCurrent(),
                                runOnce(
                                    () -> {
                                      shooterAimModel.setTurretTrim(
                                          turretTrimDefaultRot + Units.degreesToRotations(10.0));
                                    },
                                    shooterAimModel)))))
                .finallyDo(
                    () -> {
                      shooterAimModel.setTurretTrim(turretTrimDefaultRot);
                    }));

    return routine;
  }
}
