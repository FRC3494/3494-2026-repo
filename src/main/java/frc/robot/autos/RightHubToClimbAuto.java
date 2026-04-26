package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class RightHubToClimbAuto extends AutoBase {
  @Override
  public String getName() {
    return "RightHubToClimb";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.RightHubStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(routineName);

    AutoTrajectory rightHubToClimb_0 =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightHubToClimb_BLUE.segment(0).asAutoTraj(routine)
            : ChoreoTraj.RightHubToClimb_RED.segment(0).asAutoTraj(routine);
    AutoTrajectory rightHubToClimb_1 =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightHubToClimb_BLUE.segment(1).asAutoTraj(routine)
            : ChoreoTraj.RightHubToClimb_RED.segment(1).asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                rightHubToClimb_0.resetOdometry(),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    rightHubToClimb_0.cmd())));

    rightHubToClimb_0
        .done()
        .onTrue(
            sequence(
                requirements.robotCommands().stopDrive(),
                requirements.robotCommands().dropIntakeWithSpin(),
                rightHubToClimb_1.cmd()));

    rightHubToClimb_1
        .done()
        .onTrue(
            sequence(
                requirements
                    .robotCommands()
                    .runClimberUp()
                    .deadlineFor(requirements.robotCommands().shoot()),
                requirements.robotCommands().shoot().withTimeout(2),
                Autos.climbOutpost(requirements)));

    return routine;
  }
}
