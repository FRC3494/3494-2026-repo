package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.climbPoseDepot;

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

public class DepotAndClimbAuto {
  public static AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(name);

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
                parallel(robotCommands.dropIntake()),
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
                    robotCommands.shoot(),
                    sequence(
                        new AutoAlignCommand(climbPoseDepot, drive), robotCommands.creepBackward()),
                    sequence(
                        waitUntil(() -> Timer.getMatchTime() <= 3),
                        robotCommands.climberMidWithCurrent(),
                        runOnce(
                            () -> {
                              shooterAimModel.setTurretTrim(Units.degreesToRotations(-10.0));
                            },
                            shooterAimModel)))));

    return routine;
  }
}
