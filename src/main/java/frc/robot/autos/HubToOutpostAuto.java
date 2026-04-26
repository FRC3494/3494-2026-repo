package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.alliance;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class HubToOutpostAuto extends AutoBase {
  @Override
  public String getName() {
    return "HubToOutpost";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.RightHubStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(routineName);

    AutoTrajectory rightHubToOutpost_0 =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightHubToOutpost_BLUE.segment(0).asAutoTraj(routine)
            : ChoreoTraj.RightHubToOutpost_RED.segment(0).asAutoTraj(routine);
    AutoTrajectory rightHubToOutpost_1 =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightHubToOutpost_BLUE.segment(1).asAutoTraj(routine)
            : ChoreoTraj.RightHubToOutpost_RED.segment(1).asAutoTraj(routine);
    AutoTrajectory outpostToRightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.OutpostToRightClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.OutpostToRightClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                rightHubToOutpost_0.resetOdometry(),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    rightHubToOutpost_0.cmd())));

    rightHubToOutpost_0
        .done()
        .onTrue(
            sequence(
                requirements.robotCommands().stopDrive(),
                requirements.robotCommands().dropIntakeWithSpin(),
                rightHubToOutpost_1.cmd()));

    rightHubToOutpost_1
        .done()
        .onTrue(
            sequence(
                requirements
                    .robotCommands()
                    .runClimberUp()
                    .deadlineFor(requirements.robotCommands().shoot()),
                outpostToRightClimb.cmd().deadlineFor(requirements.robotCommands().shoot())));

    outpostToRightClimb.done().onTrue(Autos.climbOutpost(requirements));

    return routine;
  }
}
