package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.choreo.ChoreoTraj;

public class TestAuto {
  public static AutoRoutine getRoutine(String name, AutoFactory autoFactory, Drive drive) {
    AutoRoutine routine = autoFactory.newRoutine(name);

    AutoTrajectory traj1 = ChoreoTraj.NewPath_copy2.asAutoTraj(routine);
    AutoTrajectory traj2 = ChoreoTraj.NewPath_copy1.asAutoTraj(routine);

    routine.active().onTrue(sequence(traj1.resetOdometry(), traj1.cmd()));
    traj1.done().onTrue(sequence(print("Doneeeeee with #1"), waitSeconds(3)));

    traj2.done().onTrue(sequence(waitSeconds(1), print("Traj2 done")));

    return routine;
  }
}
