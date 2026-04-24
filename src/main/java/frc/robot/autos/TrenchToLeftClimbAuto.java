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

public class TrenchToLeftClimbAuto extends AutoBase {
  public String getName() {
    return "TrenchToLeftClimb";
  }

  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftTrenchStartingPosition;
  }

  public AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory trenchToLeftClimb = ChoreoTraj.TrenchToLeftClimb.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                trenchToLeftClimb.resetOdometry(),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    trenchToLeftClimb.cmd())));

    trenchToLeftClimb
        .atTime("StartFlywheel")
        .onTrue(robotCommands.startFlywheel().andThen(print("StartFlywheel")));

    trenchToLeftClimb
        .done()
        .onTrue(
            sequence(
                    robotCommands
                        .runClimberUp()
                        .deadlineFor(robotCommands.shootWithoutIntakeJostle()),
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
                                .deadlineFor(robotCommands.shootWithoutIntakeJostle()),
                            runOnce(
                                () -> {
                                  shooterAimModel.setTurretTrim(
                                      turretTrimDefaultRot + Units.degreesToRotations(-5.0));
                                },
                                shooterAimModel),
                            parallel(robotCommands.runClimberMidWithCurrent()),
                            runOnce(
                                () -> {
                                  shooterAimModel.setTurretTrim(
                                      turretTrimDefaultRot + Units.degreesToRotations(-10.0));
                                },
                                shooterAimModel),
                            robotCommands.shootWithoutIntakeJostle())))
                .finallyDo(
                    () -> {
                      shooterAimModel.setTurretTrim(turretTrimDefaultRot);
                    }));

    return routine;
  }
}
