package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Command;

public class SetFlywheelCommand extends Command {
  public static Command getCommand(Flywheel flywheel, double rpm) {
    return runOnce(() -> flywheel.setVelocity(RPM.of(rpm)), flywheel);
  }
}
