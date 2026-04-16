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

public class RightNZToClimbAuto {
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
    AutoTrajectory middleNZToRightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.MiddleNZToRightClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.MiddleNZToRightClimb_RED.asAutoTraj(routine);

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

    rightTrenchToNZ.done().onTrue(sequence(robotCommands.stopIntake(), middleNZToRightClimb.cmd()));

    middleNZToRightClimb
        .atPose("ClimberUp", Units.inchesToMeters(6), Math.PI)
        .onTrue(robotCommands.startClimberUp().andThen(print("ClimberUp")));

    middleNZToRightClimb
        .atTime("StartFlywheel")
        .onTrue(robotCommands.startFlywheel().andThen(print("StartFlywheel")));

    middleNZToRightClimb
        .done()
        .onTrue(
            sequence(
                    robotCommands.runClimberUp().deadlineFor(robotCommands.shoot()),
                    parallel(
                        robotCommands.shoot(),
                        sequence(
                            AutoAlignCommand.alignSequence(
                                drive, climbSetupPoseOutpost, climbPoseOutpost),
                            robotCommands.creepBackward()),
                        sequence(
                            waitUntil(() -> Timer.getMatchTime() <= 4),
                            robotCommands.runClimberMidWithCurrent(),
                            runOnce(
                                () -> {
                                  shooterAimModel.setTurretTrim(
                                      turretTrimDefaultRot + Units.degreesToRotations(10.0));
                                },
                                shooterAimModel))))
                .finallyDo(
                    () -> {
                      shooterAimModel.setTurretTrim(turretTrimDefaultRot);
                    }));

    return routine;
  }
}
