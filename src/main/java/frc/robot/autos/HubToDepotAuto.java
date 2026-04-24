package frc.robot.autos;

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

    AutoTrajectory leftHubToDepot = ChoreoTraj.LeftHubToDepot.asAutoTraj(routine);
    AutoTrajectory depotIntake = ChoreoTraj.DepotIntake.asAutoTraj(routine);
    AutoTrajectory depotIntakeToClimb = ChoreoTraj.DepotIntakeToClimb.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftHubToDepot.resetOdometry(),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    leftHubToDepot.cmd())));

    leftHubToDepot
        .done()
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(2),
                depotIntake.cmd().deadlineFor(robotCommands.intake())));

    depotIntake
        .done()
        .onTrue(
            sequence(
                robotCommands.runClimberUp().deadlineFor(robotCommands.shoot()),
                depotIntakeToClimb.cmd()));

    depotIntakeToClimb.done().onTrue(Autos.climbDepot(robotCommands, drive, shooterAimModel));

    return routine;
  }
}
