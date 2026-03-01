package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.Constants.ClimberConstants.climberCurrentLimit;
import static frc.robot.Constants.ClimberConstants.climberMinPosition;

import edu.wpi.first.wpilibj2.command.Command;

public class RezeroClimberCommand {
  public static Command getCommand(Climber climber) {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(Amps.of(20));
              climber.setOpenLoop(Volts.of(2));
            },
            climber),
        waitUntil(() -> climber.getFilteredCurrent().gte(Amps.of(19))),
        runOnce(
            () -> {
              climber.setOpenLoop(Volts.of(0));
              climber.setRelativeEncoderPosition(climberMinPosition);
              climber.setCurrentLimit(Amps.of(climberCurrentLimit));
            },
            climber));
  }
}
