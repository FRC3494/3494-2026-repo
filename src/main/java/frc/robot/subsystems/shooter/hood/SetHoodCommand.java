package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SetHoodCommand {
  public static Command getCommand(Hood hood, double angle) {
    return runOnce(
        () -> {
          hood.setPosition(new Rotation2d(angle));
        },
        hood);
  }
}
