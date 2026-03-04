package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.RobotCommands;
import frc.robot.util.choreo.ChoreoTraj;

public class DepotAndClimbAuto {
  public static AutoRoutine getRoutine(
      String name, AutoFactory autoFactory, RobotCommands robotCommands) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory leftBumpToDepot = ChoreoTraj.LeftBumpToDepot.asAutoTraj(routine);
    AutoTrajectory depotToLeftClimb = ChoreoTraj.DepotToLeftClimb.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            parallel(
                robotCommands.climberUp(),
                sequence(
                    leftBumpToDepot.resetOdometry(),
                    robotCommands.setCloseShot(),
                    robotCommands.shoot(),
                    waitSeconds(3),
                    robotCommands.spinDownFromShoot(),
                    parallel(robotCommands.intake(), leftBumpToDepot.cmd()))));

    leftBumpToDepot
        .done()
        .onTrue(
            sequence(
                robotCommands.stopDrive(),
                print("done with left bump to depot"),
                robotCommands.creepForward().withTimeout(0.4),
                robotCommands.stopDrive(),
                waitSeconds(0.5),
                robotCommands.releaseIntake(),
                robotCommands.setMediumShot(),
                robotCommands.shoot(),
                waitSeconds(2),
                robotCommands.spinDownFromShoot(),
                depotToLeftClimb.cmd()));

    depotToLeftClimb
        .done()
        .onTrue(
            sequence(
                print("done with depot to left climb"),
                parallel(
                    robotCommands.creepBackward(),
                    sequence(waitSeconds(1), robotCommands.climberMid()))));

    return routine;
  }
}
