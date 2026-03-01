package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.choreo.ChoreoTraj;

public class ClimbLeftAuto {
  public static AutoRoutine getRoutine(
      String name, AutoFactory autoFactory, Climber climber, Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory driveToTower = ChoreoTraj.ClimbLeft.asAutoTraj(routine);

    routine
        .active()
        .onTrue(
            sequence(
                print("Routine.active() -------------"),
                driveToTower.resetOdometry(),
                print("Done with resetOdometry ----------------"),
                driveToTower.cmd(),
                print("Done with cmd() ---------------------------")));

    driveToTower.done().onTrue(print("Done with auto ClimbLeft ----------------------"));

    return routine;
  }
}
