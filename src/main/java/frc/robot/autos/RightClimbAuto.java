package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class RightClimbAuto extends AutoBase {
  @Override
  public String getName() {
    return "RightClimb";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.RightTrenchStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(String name, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(name);

    AutoTrajectory rightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightTrenchToClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightTrenchToClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                rightClimb.resetOdometry(),
                requirements.robotCommands().enableAutoShooterSettings(),
                requirements.robotCommands().enableAutoTurret(),
                waitSeconds(0.5),
                requirements.robotCommands().runClimberUp(),
                requirements.robotCommands().shoot().withTimeout(3),
                parallel(rightClimb.cmd(), requirements.robotCommands().spinDownFromShoot())));

    rightClimb.done().onTrue(Autos.climbOutpost(requirements));

    return routine;
  }
}
