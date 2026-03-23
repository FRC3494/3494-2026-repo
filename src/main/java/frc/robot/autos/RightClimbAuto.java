package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.climbPoseOutpost;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.util.choreo.ChoreoTraj;

public class RightClimbAuto {
  public static AutoRoutine getRoutine(
      String name, AutoFactory autoFactory, RobotCommands robotCommands, Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory rightClimb = ChoreoTraj.RightClimb.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                rightClimb.resetOdometry(),
                robotCommands.enableAutoShooterSettings(),
                robotCommands.enableAutoTurret(),
                waitSeconds(0.5),
                robotCommands.climberUp(),
                robotCommands.shoot(),
                waitSeconds(3),
                parallel(rightClimb.cmd(), robotCommands.spinDownFromShoot())));

    rightClimb
        .done()
        .onTrue(
            parallel(
                sequence(
                    new AutoAlignCommand(climbPoseOutpost, drive), robotCommands.creepBackward()),
                sequence(waitSeconds(1), robotCommands.climberMid())));

    return routine;
  }
}
