package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class RightOutpostAuto extends AutoBase {
  @Override
  public String getName() {
    return "RightOutpost";
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
    AutoTrajectory rightMiddleNZToOutpost_0 =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightMiddleNZToOutpost_BLUE.segment(0).asAutoTraj(routine)
            : ChoreoTraj.RightMiddleNZToOutpost_RED.segment(0).asAutoTraj(routine);
    AutoTrajectory rightMiddleNZToOutpost_1 =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightMiddleNZToOutpost_BLUE.segment(1).asAutoTraj(routine)
            : ChoreoTraj.RightMiddleNZToOutpost_RED.segment(1).asAutoTraj(routine);
    AutoTrajectory outpostToRightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.OutpostToRightClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.OutpostToRightClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                rightTrenchToNZ.resetOdometry(),
                parallel(
                    requirements.robotCommands().enableAutoShooterSettings(),
                    requirements.robotCommands().enableAutoTurret(),
                    rightTrenchToNZ.cmd())));

    rightTrenchToNZ.atTime("NZIntake").onTrue(requirements.robotCommands().intake());

    rightTrenchToNZ
        .done()
        .onTrue(
            sequence(requirements.robotCommands().stopIntake(), rightMiddleNZToOutpost_0.cmd()));

    rightMiddleNZToOutpost_0
        .atTime("StartFlywheel")
        .onTrue(requirements.robotCommands().startFlywheel());

    rightMiddleNZToOutpost_0
        .done()
        .onTrue(rightMiddleNZToOutpost_1.cmd().deadlineFor(requirements.robotCommands().shoot()));

    rightMiddleNZToOutpost_1
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
