package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.Constants.ShooterConstants.HoodConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class RezeroHoodCommand {
  public static Command getCommand(Hood hood) {
    return sequence(
        runOnce(
            () -> {
              hood.setCurrentLimit(Amps.of(20));
              hood.setOpenLoop(Volts.of(-1));
            },
            hood),
        waitUntil(() -> hood.getFilteredCurrent().gte(Amps.of(19))),
        runOnce(
            () -> {
              hood.setOpenLoop(Volts.of(0));
              hood.setRelativeEncoderPosition(hoodMinAngle);
              hood.setCurrentLimit(Amps.of(hoodCurrentLimit));
            },
            hood)).withTimeout(hoodRezeroTimeoutSeconds);
  }
}
