package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class WarmUpAuto extends AutoBase {
  @Override
  public String getName() {
    return "WarmUp";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.WarmUpPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(routineName);

    AutoTrajectory warmUp = ChoreoTraj.WarmUp.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                    warmUp.resetOdometry(),
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().turretToPosition(Units.degreesToRotations(1)),
                    requirements.robotCommands().runClimberUp(),
                    waitSeconds(0.25),
                    requirements.robotCommands().runClimberDown(),
                    requirements.robotCommands().turretToPosition(Units.degreesToRotations(15)),
                    requirements.robotCommands().shoot().withTimeout(3),
                    requirements.robotCommands().stopShootNoDelay(),
                    waitSeconds(0.5),
                    warmUp.cmd())
                .finallyDo(
                    () -> {
                      CommandScheduler.getInstance()
                          .schedule(requirements.robotCommands().enableAutoTurret());
                    }));

    return routine;
  }
}
