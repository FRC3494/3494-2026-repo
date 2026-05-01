package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class NZToDepotAuto extends AutoBase {
  @Override
  public String getName() {
    return "NZToDepot";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftTrenchStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName, Alliance alliance, AutoRequirements requirements) {
    AutoRoutine routine = requirements.autoFactory().newRoutine(routineName);

    AutoTrajectory leftTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory leftMiddleNZToDepot_0 =
        ChoreoTraj.LeftMiddleNZToDepot.segment(0).asAutoTraj(routine);
    AutoTrajectory leftMiddleNZToDepot_1 =
        ChoreoTraj.LeftMiddleNZToDepot.segment(1).asAutoTraj(routine);
    AutoTrajectory depotIntake =
        alliance == Alliance.Blue
            ? ChoreoTraj.DepotIntake_BLUE.asAutoTraj(routine)
            : ChoreoTraj.DepotIntake_RED.asAutoTraj(routine);
    AutoTrajectory depotIntakeToClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.DepotIntakeToClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.DepotIntakeToClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftTrenchToNZ.resetOdometry(),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    leftTrenchToNZ.cmd())));

    leftTrenchToNZ
        .atTime("NZIntake")
        .onTrue(requirements.robotCommands().intake().andThen(print("NZIntake")));

    leftTrenchToNZ
        .done()
        .onTrue(sequence(requirements.robotCommands().stopIntake(), leftMiddleNZToDepot_0.cmd()));

    leftMiddleNZToDepot_0
        .atTime("StartFlywheel")
        .onTrue(requirements.robotCommands().startFlywheel());

    leftMiddleNZToDepot_0
        .done()
        .onTrue(leftMiddleNZToDepot_1.cmd().deadlineFor(requirements.robotCommands().shoot()));

    leftMiddleNZToDepot_1
        .done()
        .onTrue(
            sequence(
                parallel(
                    requirements.robotCommands().runClimberUp(),
                    depotIntake.cmd().deadlineFor(requirements.robotCommands().intake()))));

    depotIntake.done().onTrue(depotIntakeToClimb.cmd());

    depotIntakeToClimb.done().onTrue(Autos.climbDepot(requirements));

    return routine;
  }
}
