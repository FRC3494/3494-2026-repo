package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class LeftNZToClimbAuto extends AutoBase {
  @Override
  public String getName() {
    return "LeftNZToClimb";
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
    AutoTrajectory middleNZToLeftClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.MiddleNZToLeftClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.MiddleNZToLeftClimb_RED.asAutoTraj(routine);

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
        .onTrue(sequence(requirements.robotCommands().stopIntake(), middleNZToLeftClimb.cmd()));

    // middleNZToLeftClimb
    //     .atPose("ClimberUp", Units.inchesToMeters(12), Math.PI)
    //     .onTrue(
    //         requirements.robotCommands()
    //             .startClimberUp()
    //             .andThen(
    //                 print(
    //                     "ClimberUp")));

    middleNZToLeftClimb
        .atTime("StartFlywheel")
        .onTrue(requirements.robotCommands().startFlywheel().andThen(print("StartFlywheel")));

    middleNZToLeftClimb
        .done()
        .onTrue(
            sequence(
                    requirements
                        .robotCommands()
                        .runClimberUp()
                        .deadlineFor(requirements.robotCommands().shoot()),
                    Autos.climbDepot(requirements))
                .finallyDo(
                    () -> {
                      requirements.shooterAimModel().setTurretTrim(turretTrimDefaultRot);
                    }));

    return routine;
  }
}
