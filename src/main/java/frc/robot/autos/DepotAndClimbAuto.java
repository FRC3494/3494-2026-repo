package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.climbPoseDepot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.util.choreo.ChoreoTraj;

public class DepotAndClimbAuto {
  public static AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive) {
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
                waitSeconds(0.5),
                robotCommands.climberUp(),
                robotCommands.shoot(),
                waitSeconds(3),
                parallel(robotCommands.spinDownFromShoot(), robotCommands.dropIntake()),
                parallel(robotCommands.intake(), leftBumpToDepotPartial.cmd())));

    leftBumpToDepotPartial
        .done()
        .onTrue(sequence(robotCommands.shoot(), waitSeconds(1.75), depotToLeftClimb.cmd()));

    depotToLeftClimb
        .done()
        .onTrue(
            sequence(
                parallel(
                    robotCommands.spinDownFromShoot(),
                    sequence(
                        new AutoAlignCommand(climbPoseDepot, drive), robotCommands.creepBackward()),
                    sequence(waitSeconds(1), robotCommands.climberMid()))));

    return routine;
  }
}
