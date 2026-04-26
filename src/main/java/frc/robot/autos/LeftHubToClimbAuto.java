package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class LeftHubToClimbAuto extends AutoBase {
  @Override
  public String getName() {
    return "LeftHubToClimb";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftHubStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(routineName);

    AutoTrajectory leftHubToClimb_0 =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftHubToClimb_BLUE.segment(0).asAutoTraj(routine)
            : ChoreoTraj.LeftHubToClimb_RED.segment(0).asAutoTraj(routine);
    AutoTrajectory leftHubToClimb_1 =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftHubToClimb_BLUE.segment(1).asAutoTraj(routine)
            : ChoreoTraj.LeftHubToClimb_RED.segment(1).asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftHubToClimb_0.resetOdometry(),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    leftHubToClimb_0.cmd())));

    leftHubToClimb_0
        .done()
        .onTrue(
            sequence(
                requirements.robotCommands().stopDrive(),
                requirements.robotCommands().dropIntakeWithSpin(),
                leftHubToClimb_1.cmd()));

    leftHubToClimb_1
        .done()
        .onTrue(
            sequence(
                requirements
                    .robotCommands()
                    .runClimberUp()
                    .deadlineFor(requirements.robotCommands().shoot()),
                requirements.robotCommands().shoot().withTimeout(2),
                Autos.climbDepot(requirements)));

    return routine;
  }
}
