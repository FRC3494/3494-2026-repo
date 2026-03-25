package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.choreo.ChoreoTraj;

public class LeftNZToNZAuto {
  public static AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory leftTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory middleNZToShoot =
        alliance == Alliance.Blue
            ? ChoreoTraj.MiddleNZToShoot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.MiddleNZToShoot_RED.asAutoTraj(routine);
    AutoTrajectory leftShootToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftShootToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftShootToNZ_RED.asAutoTraj(routine);

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
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(10),
                robotCommands.stopShootingNoDelay(),
                robotCommands.runIntake(),
                leftShootToNZ.cmd()));

    return routine;
  }
}
