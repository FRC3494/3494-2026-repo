package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.RobotCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.choreo.ChoreoTraj;

public class ClimbLeftAuto {
  public static AutoRoutine getRoutine(
      String name, AutoFactory autoFactory, RobotCommands robotCommands, Climber climber) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory driveToTower = ChoreoTraj.ClimbLeft.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            parallel(
                sequence(
                    print("Routine.active() -------------"),
                    driveToTower.resetOdometry(),
                    print("Done with resetOdometry ----------------"),
                    driveToTower.cmd(),
                    print("Done with cmd() ---------------------------")),
                robotCommands.climberUp()));

    driveToTower
        .done()
        .onTrue(
            sequence(
                print("Done with traj driveToTower ----------------------"),
                robotCommands.climberMid()));

    return routine;
  }
}
