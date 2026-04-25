package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class HubToDepotAuto extends AutoBase {
  @Override
  public String getName() {
    return "HubToDepot";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftHubStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(routineName);

    AutoTrajectory leftHubToDepot_0 =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftHubToDepot_BLUE.segment(0).asAutoTraj(routine)
            : ChoreoTraj.LeftHubToDepot_RED.segment(0).asAutoTraj(routine);
    AutoTrajectory leftHubToDepot_1 =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftHubToDepot_BLUE.segment(1).asAutoTraj(routine)
            : ChoreoTraj.LeftHubToDepot_RED.segment(1).asAutoTraj(routine);
    AutoTrajectory depotIntake =
        alliance == Alliance.Blue
            ? ChoreoTraj.DepotIntake_BLUE.asAutoTraj(routine)
            : ChoreoTraj.DepotIntake_RED.asAutoTraj(routine);
    AutoTrajectory depotIntakeToClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.DepotIntakeToClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.DepotIntakeToClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftHubToDepot_0.resetOdometry(),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    leftHubToDepot_0.cmd())));

    leftHubToDepot_0
        .done()
        .onTrue(
            sequence(
                requirements.robotCommands().stopDrive(),
                requirements.robotCommands().dropIntakeWithSpin(),
                leftHubToDepot_1.cmd()));

    leftHubToDepot_1
        .done()
        .onTrue(
            sequence(
                parallel(
                    requirements.robotCommands().runClimberUp(),
                    depotIntake.cmd().deadlineFor(requirements.robotCommands().intake()))));

    depotIntake.done().onTrue(depotIntakeToClimb.cmd());

    depotIntakeToClimb.done().onTrue(Autos.climbDepot(requirements));

    return routine;
  }
}
