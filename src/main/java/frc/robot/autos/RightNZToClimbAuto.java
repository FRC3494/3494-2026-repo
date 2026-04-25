package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class RightNZToClimbAuto extends AutoBase {
  @Override
  public String getName() {
    return "RightNZToClimb";
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
    AutoTrajectory middleNZToRightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.MiddleNZToRightClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.MiddleNZToRightClimb_RED.asAutoTraj(routine);

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
                    rightTrenchToNZ.cmd()),
                print("3")));

    rightTrenchToNZ.atTime("NZIntake").onTrue(requirements.robotCommands().intake());

    rightTrenchToNZ
        .done()
        .onTrue(sequence(requirements.robotCommands().stopIntake(), middleNZToRightClimb.cmd()));

    middleNZToRightClimb
        .atPose("ClimberUp", Units.inchesToMeters(6), Math.PI)
        .onTrue(requirements.robotCommands().startClimberUp().andThen(print("ClimberUp")));

    middleNZToRightClimb
        .atTime("StartFlywheel")
        .onTrue(requirements.robotCommands().startFlywheel().andThen(print("StartFlywheel")));

    middleNZToRightClimb
        .done()
        .onTrue(
            sequence(
                    requirements
                        .robotCommands()
                        .runClimberUp()
                        .deadlineFor(requirements.robotCommands().shoot()),
                    Autos.climbOutpost(requirements))
                .finallyDo(
                    () -> {
                      requirements.shooterAimModel().setTurretTrim(turretTrimDefaultRot);
                    }));

    return routine;
  }
}
