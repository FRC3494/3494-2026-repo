package frc.robot.autos;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;

public abstract class AutoBase {
  public abstract String getName();

  public abstract Pose2d getStartingPose();

  public abstract AutoRoutine getRoutine(
      String routineName, Alliance alliance, AutoRequirements requirements);
}
