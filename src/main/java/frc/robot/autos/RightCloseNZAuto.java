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

public class RightCloseNZAuto {
  public static AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory rightTrenchToCloseNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightTrenchToCloseNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightTrenchToCloseNZ_RED.asAutoTraj(routine);
    AutoTrajectory rightMiddleNZToShoot =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightMiddleNZToShoot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightMiddleNZToShoot_RED.asAutoTraj(routine);
    AutoTrajectory rightShootToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightShootToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightShootToNZ_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("1"),
                rightTrenchToCloseNZ.resetOdometry(),
                waitSeconds(0),
                print("2"),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    rightTrenchToCloseNZ.cmd()),
                print("3")));

    rightTrenchToCloseNZ.atTime("NZIntake").onTrue(robotCommands.intake());

    rightTrenchToCloseNZ
        .done()
        .onTrue(sequence(robotCommands.stopIntake(), rightMiddleNZToShoot.cmd()));

    rightMiddleNZToShoot.atTime("StartFlywheel").onTrue(robotCommands.startFlywheel());

    rightMiddleNZToShoot
        .done()
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(10),
                robotCommands.stopShootingNoDelay(),
                robotCommands.runIntake(),
                rightShootToNZ.cmd()));

    return routine;
  }
}
