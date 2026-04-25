package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;

public class WarmUpAuto extends AutoBase {
  @Override
  public String getName() {
    return "WarmUp";
  }

  @Override
  public Pose2d getStartingPose() {
    return Pose2d.kZero;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(routineName);

    AutoTrajectory warmUpTurn = ChoreoTraj.WarmUpTurn.asAutoTraj(routine);
    AutoTrajectory warmUpStraight = ChoreoTraj.WarmUpStraight.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                warmUpTurn.resetOdometry(),
                requirements.robotCommands().runClimberUp(),
                waitSeconds(0.5),
                requirements.robotCommands().runClimberDown(),
                requirements.robotCommands().shoot().withTimeout(3),
                requirements.robotCommands().stopShootNoDelay(),
                warmUpTurn.cmd()));

    warmUpTurn.chain(warmUpStraight);

    return routine;
  }
}
