package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.generated.ChoreoTraj;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.intake.SpinnySpinny;

public class RightNZToNZAuto extends AutoBase {
  @Override
  public String getName() {
    return "RightNZToNZ";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.RightTrenchStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(String name, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(name);

    AutoTrajectory rightTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory rightMiddleNZToShoot =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightMiddleNZToShoot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightMiddleNZToShoot_RED.asAutoTraj(routine);
    AutoTrajectory rightShootToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightShootToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightShootToNZ_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("1"),
                rightTrenchToNZ.resetOdometry(),
                print("2"),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    requirements.robotCommands().runIntakeDownFast(),
                    rightTrenchToNZ.cmd()),
                print("3")));

    rightTrenchToNZ
        .atTime("NZIntake")
        .onTrue(requirements.spinnySpinny().setState(SpinnySpinny.State.INTAKE));

    rightTrenchToNZ
        .done()
        .onTrue(
            sequence(
                requirements.spinnySpinny().setState(SpinnySpinny.State.IDLE),
                rightMiddleNZToShoot.cmd()));

    rightMiddleNZToShoot
        .atTime("StartFlywheel")
        .onTrue(requirements.robotCommands().startFlywheel());

    rightMiddleNZToShoot
        .done()
        .onTrue(
            sequence(
                requirements.robotCommands().shoot().withTimeout(7),
                requirements.robotCommands().stopShootNoDelay(),
                requirements.spinnySpinny().setState(SpinnySpinny.State.INTAKE),
                parallel(requirements.robotCommands().runIntakeDownFast(), rightShootToNZ.cmd())));

    return routine;
  }
}
