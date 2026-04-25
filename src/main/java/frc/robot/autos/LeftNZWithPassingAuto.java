package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class LeftNZWithPassingAuto extends AutoBase {
  @Override
  public String getName() {
    return "LeftNZWithPassing";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftTrenchStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(String name, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(name);

    AutoTrajectory leftTrenchToFarNZ = ChoreoTraj.LeftTrenchToFarNZ.asAutoTraj(routine);
    AutoTrajectory leftNZSecondPass = ChoreoTraj.LeftNZSecondPass.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftTrenchToFarNZ.resetOdometry(),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    leftTrenchToFarNZ.cmd())));

    leftTrenchToFarNZ.atTime("NZIntake").onTrue(requirements.robotCommands().shoot());

    leftTrenchToFarNZ
        .done()
        .onTrue(
            sequence(
                requirements.robotCommands().stopShootNoDelay(),
                leftNZSecondPass.cmd().deadlineFor(requirements.robotCommands().intake())));

    leftNZSecondPass.done().onTrue(requirements.robotCommands().shoot());

    return routine;
  }
}
