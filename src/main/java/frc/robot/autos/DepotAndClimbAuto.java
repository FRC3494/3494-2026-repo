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

public class DepotAndClimbAuto extends AutoBase {
  @Override
  public String getName() {
    return "DepotAndClimb";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftBumpStartingPosition;
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

    AutoTrajectory leftBumpToDepot =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftBumpToDepot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftBumpToDepot_RED.asAutoTraj(routine);
    AutoTrajectory leftBumpToDepotPartial =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftBumpToDepotPartial_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftBumpToDepotPartial_RED.asAutoTraj(routine);
    AutoTrajectory depotToLeftClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.DepotToLeftClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.DepotToLeftClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftBumpToDepot.resetOdometry(),
                robotCommands.enableAutoShooterSettings(),
                robotCommands.enableAutoTurret(),
                robotCommands.startClimberUp(),
                robotCommands.dropIntakeWithDrive(),
                leftBumpToDepotPartial.cmd().deadlineFor(robotCommands.intake())));

    leftBumpToDepotPartial
        .done()
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(1.75),
                depotToLeftClimb.cmd().deadlineFor(robotCommands.shoot())));

    depotToLeftClimb
        .done()
        .onTrue(
            sequence(
                    parallel(
                        sequence(
                            new AutoAlignCommand(
                                alliance == Alliance.Blue
                                    ? climbPoseDepot_BLUE
                                    : climbPoseDepot_RED,
                                drive),
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
}
