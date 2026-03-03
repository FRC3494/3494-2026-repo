package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Command;

public class RezeroTurretCommand {
  public static Command getCommand(Turret turret) {
    return runOnce(
        () -> {
          turret.rezeroFromAbsEncoder();
        },
        turret);
  }
}
