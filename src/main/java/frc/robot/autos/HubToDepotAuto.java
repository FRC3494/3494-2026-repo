package frc.robot.autos;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;

public class HubToDepotAuto extends AutoBase {
  @Override
  public String getName() {
    return "HubToDepot";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.LeftHubStartingPosition;
  }

  @Override
  public AutoRoutine getRoutine(
      String routineName,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(routineName);

    AutoTrajectory leftHubToDepot_0 = ChoreoTraj.LeftHubToDepot.segment(0).asAutoTraj(routine);
    AutoTrajectory leftHubToDepot_1 = ChoreoTraj.LeftHubToDepot.segment(1).asAutoTraj(routine);
    AutoTrajectory depotIntake = ChoreoTraj.DepotIntake.asAutoTraj(routine);
    AutoTrajectory depotIntakeToClimb = ChoreoTraj.DepotIntakeToClimb.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftHubToDepot_0.resetOdometry(),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    leftHubToDepot_0.cmd())));

    leftHubToDepot_0
        .done()
        .onTrue(
            sequence(
                robotCommands.stopDrive(),
                robotCommands.dropIntakeWithSpin(),
                leftHubToDepot_1.cmd()));

    leftHubToDepot_1
        .done()
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(1),
                robotCommands.stopShootNoDelay(),
                parallel(
                    robotCommands.runClimberUp(),
                    depotIntake.cmd().deadlineFor(robotCommands.intake()))));

    depotIntake.done().onTrue(depotIntakeToClimb.cmd());

    depotIntakeToClimb.done().onTrue(Autos.climbDepot(robotCommands, drive, shooterAimModel));

    return routine;
  }
}
