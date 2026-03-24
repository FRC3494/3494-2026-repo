package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.choreo.ChoreoTraj;

public class LeftNZToNZAuto {
  public static AutoRoutine getRoutine(
      String name, AutoFactory autoFactory, RobotCommands robotCommands, Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory leftTrenchToNZ = ChoreoTraj.LeftTrenchToNZ.asAutoTraj(routine);
    AutoTrajectory middleNZToShoot = ChoreoTraj.MiddleNZToShoot.asAutoTraj(routine);
    AutoTrajectory leftShootToNZ = ChoreoTraj.LeftShootToNZ.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("1"),
                leftTrenchToNZ.resetOdometry(),
                print("2"),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    leftTrenchToNZ.cmd()),
                print("3")));

    leftTrenchToNZ.atTime("LeftNZIntake").onTrue(robotCommands.intake());

    leftTrenchToNZ.done().onTrue(sequence(robotCommands.stopIntake(), middleNZToShoot.cmd()));

    middleNZToShoot
        .done()
        .onTrue(sequence(robotCommands.shoot().withTimeout(10), leftShootToNZ.cmd()));

    leftShootToNZ
        .atTime("LeftNZIntake")
        .onTrue(sequence(robotCommands.spinDownFromShoot(), robotCommands.intake()));

    return routine;
  }
}
