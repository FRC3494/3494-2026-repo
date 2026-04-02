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

public class RightNZLoopAuto {
  public static AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory rightTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory rightNZLoop =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightNZLoop_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightNZLoop_RED.asAutoTraj(routine);
    AutoTrajectory rightNZLoopToShoot =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightNZLoopToShoot_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightNZLoopToShoot_RED.asAutoTraj(routine);
    AutoTrajectory rightShootToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightShootToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightShootToNZ_RED.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("1"),
                rightTrenchToNZ.resetOdometry(),
                print("2"),
                parallel(
                    robotCommands.enableAutoShooterSettings(),
                    robotCommands.enableAutoTurret(),
                    rightTrenchToNZ.cmd()),
                print("3")));

    rightTrenchToNZ.atTime("LeftNZIntake").onTrue(robotCommands.intake());

    rightTrenchToNZ.done().onTrue(rightNZLoop.cmd());

    rightNZLoop.done().onTrue(sequence(robotCommands.stopIntake(), rightNZLoopToShoot.cmd()));

    rightNZLoopToShoot
        .done()
        .onTrue(
            sequence(
                robotCommands.shoot().withTimeout(10),
                robotCommands.stopShootNoDelay(),
                robotCommands.startIntake(),
                rightShootToNZ.cmd()));

    return routine;
  }
}
