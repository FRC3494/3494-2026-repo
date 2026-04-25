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

    AutoTrajectory leftHubToClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftHubToClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftHubToClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftHubToClimb.resetOdometry(),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    leftHubToClimb.cmd())));

    leftHubToClimb
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
