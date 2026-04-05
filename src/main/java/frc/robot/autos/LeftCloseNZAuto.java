package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.choreo.ChoreoTraj;

public class LeftCloseNZAuto {
  public static AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory leftTrenchToCloseNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftTrenchToCloseNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftTrenchToCloseNZ_RED.asAutoTraj(routine);
    AutoTrajectory leftMiddleNZToShoot =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftMiddleNZToShoot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftMiddleNZToShoot_RED.asAutoTraj(routine);
    AutoTrajectory leftShootToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.LeftShootToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.LeftShootToNZ_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("1"),
                leftTrenchToCloseNZ.resetOdometry(),
                waitSeconds(0),
                print("2"),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    leftTrenchToCloseNZ.cmd()),
                print("3")));

    leftTrenchToCloseNZ.atTime("NZIntake").onTrue(robotCommands.intake());

    leftTrenchToCloseNZ
        .done()
        .onTrue(sequence(robotCommands.stopIntake(), leftMiddleNZToShoot.cmd()));

    leftMiddleNZToShoot.atTime("StartFlywheel").onTrue(robotCommands.startFlywheel());

    leftMiddleNZToShoot
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
