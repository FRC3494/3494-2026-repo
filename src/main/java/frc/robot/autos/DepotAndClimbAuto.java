package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
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
      String routineName, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(routineName);

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
                requirements.robotCommands().enableAutoShooterSettings(),
                requirements.robotCommands().enableAutoTurret(),
                requirements.robotCommands().startClimberUp(),
                requirements.robotCommands().dropIntakeWithDrive(),
                leftBumpToDepotPartial.cmd().deadlineFor(requirements.robotCommands().intake())));

    leftBumpToDepotPartial
        .done()
        .onTrue(
            sequence(
                requirements.robotCommands().shoot().withTimeout(1.75),
                depotToLeftClimb.cmd().deadlineFor(requirements.robotCommands().shoot())));

    depotToLeftClimb.done().onTrue(Autos.climbDepot(requirements));

    return routine;
  }
}
