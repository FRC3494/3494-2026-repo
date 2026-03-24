package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.choreo.ChoreoTraj;

public class LeftNZAuto {
  public static AutoRoutine getRoutine(
      String name, AutoFactory autoFactory, RobotCommands robotCommands, Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory leftTrenchToNZ = ChoreoTraj.LeftTrenchToNZ.asAutoTraj(routine);
    AutoTrajectory leftNZToMiddleNZ = ChoreoTraj.LeftNZToMiddleNZ.asAutoTraj(routine);
    AutoTrajectory middleNZToShoot = ChoreoTraj.MiddleNZToShoot.asAutoTraj(routine);
    AutoTrajectory shootToLeftCorner = ChoreoTraj.ShootToLeftCorner.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                leftTrenchToNZ.resetOdometry(),
                robotCommands.enableAutoShooterSettings(),
                robotCommands.enableAutoTurret(),
                // robotCommands.dropIntake(),
                leftTrenchToNZ.cmd()));

    leftTrenchToNZ.done().onTrue(sequence(robotCommands.intake(), leftNZToMiddleNZ.cmd()));

    leftNZToMiddleNZ.done().onTrue(sequence(robotCommands.stopIntake(), middleNZToShoot.cmd()));

    middleNZToShoot
        .done()
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(12),
                parallel(robotCommands.spinDownFromShoot(), shootToLeftCorner.cmd())));

    return routine;
  }
}
