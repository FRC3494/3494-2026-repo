package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class LeftNZToNZAuto extends AutoBase {
  @Override
  public String getName() {
    return "LeftNZToNZ";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftTrenchStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(String name, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(name);

    AutoTrajectory leftTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory leftMiddleNZToShoot =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftMiddleNZToShoot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftMiddleNZToShoot_RED.asAutoTraj(routine);
    AutoTrajectory leftShootToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftShootToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftShootToNZ_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("1"),
                leftTrenchToNZ.resetOdometry(),
                print("2"),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    leftTrenchToNZ.cmd()),
                print("3")));

    leftTrenchToNZ.atTime("NZIntake").onTrue(requirements.robotCommands().intake());

    leftTrenchToNZ
        .done()
        .onTrue(sequence(requirements.robotCommands().stopIntake(), leftMiddleNZToShoot.cmd()));

    leftMiddleNZToShoot
        .atTime("StartFlywheel")
        .onTrue(requirements.robotCommands().startFlywheel());

    leftMiddleNZToShoot
        .done()
        .onTrue(
            sequence(
                requirements.robotCommands().shoot().withTimeout(7),
                requirements.robotCommands().stopShootNoDelay(),
                requirements.robotCommands().startIntake(),
                leftShootToNZ.cmd()));

    return routine;
  }
}
