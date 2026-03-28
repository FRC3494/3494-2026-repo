package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.climbPoseOutpost;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.util.choreo.ChoreoTraj;

public class RightNZToClimbAuto {
  public static AutoRoutine getRoutine(
      String name,
      Alliance alliance,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory rightTrenchToNZ =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightTrenchToNZ_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightTrenchToNZ_RED.asAutoTraj(routine);
    AutoTrajectory middleNZToRightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.MiddleNZToRightClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.MiddleNZToRightClimb_RED.asAutoTraj(routine);
    AutoTrajectory rightClimb =
        alliance == Alliance.Blue
            ? ChoreoTraj.RightClimb_BLUE.asAutoTraj(routine)
            : ChoreoTraj.RightClimb_RED.asAutoTraj(routine);

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

    rightTrenchToNZ.atTime("NZIntake").onTrue(robotCommands.intake());

    rightTrenchToNZ.done().onTrue(sequence(robotCommands.stopIntake(), middleNZToRightClimb.cmd()));

    middleNZToRightClimb
        .atPose("ClimberUp", Units.inchesToMeters(6), Math.PI)
        .onTrue(robotCommands.climberUpInstant());

    middleNZToRightClimb.atTime("StartFlywheel").onTrue(robotCommands.startFlywheel());

    middleNZToRightClimb.done().onTrue(rightClimb.cmd().deadlineFor(robotCommands.shoot()));

    rightClimb
        .done()
        .onTrue(
            sequence(
                parallel(
                    robotCommands.shoot(),
                    sequence(
                        new AutoAlignCommand(climbPoseOutpost, drive),
                        robotCommands.creepBackward()),
                    sequence(
                        waitUntil(() -> Timer.getMatchTime() <= 3),
                        robotCommands.climberMidWithCurrent(),
                        runOnce(
                            () -> {
                              shooterAimModel.setTurretTrim(Units.degreesToRotations(10.0));
                            },
                            shooterAimModel)))));

    return routine;
  }
}
