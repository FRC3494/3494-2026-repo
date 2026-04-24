package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

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
  public AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory leftTrenchToFarNZ = ChoreoTraj.LeftTrenchToFarNZ.asAutoTraj(routine);
    AutoTrajectory leftNZSecondPass = ChoreoTraj.LeftNZSecondPass.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftTrenchToFarNZ.resetOdometry(),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    leftTrenchToFarNZ.cmd())));

    leftTrenchToFarNZ.atTime("NZIntake").onTrue(robotCommands.shoot());

    leftTrenchToFarNZ
        .done()
        .onTrue(
            sequence(
                robotCommands.stopShootNoDelay(),
                leftNZSecondPass.cmd().deadlineFor(robotCommands.intake())));

    leftNZSecondPass.done().onTrue(robotCommands.shoot());

    return routine;
  }
}
