package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.Constants.alliance;

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

public class HubToOutpostAuto extends AutoBase {
  @Override
  public String getName() {
    return "HubToOutpost";
  }

  @Override
  public Pose2d getStartingPose() {
    return ChoreoVars.Poses.RightHubStartingPosition;
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

    AutoTrajectory rightHubToOutpost_0 =
        ChoreoTraj.RightHubToOutpost.segment(0).asAutoTraj(routine);
    AutoTrajectory rightHubToOutpost_1 =
        ChoreoTraj.RightHubToOutpost.segment(1).asAutoTraj(routine);
    AutoTrajectory outpostToRightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.OutpostToRightClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.OutpostToRightClimb_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                rightHubToOutpost_0.resetOdometry(),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    rightHubToOutpost_0.cmd())));

    rightHubToOutpost_0
        .done()
        .onTrue(
            sequence(
                robotCommands.stopDrive(),
                robotCommands.dropIntakeWithSpin(),
                rightHubToOutpost_1.cmd()));

    rightHubToOutpost_1
        .done()
        .onTrue(
            sequence(
                robotCommands.runClimberUp().deadlineFor(robotCommands.shoot()),
                outpostToRightClimb.cmd().deadlineFor(robotCommands.shoot())));

    outpostToRightClimb.done().onTrue(Autos.climbOutpost(robotCommands, drive, shooterAimModel));

    return routine;
  }
}
