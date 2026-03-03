package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SetTurretCommand {
  public static Command getCommand(Turret turret, double angle) {
    return runOnce(
        () -> {
          turret.setPosition(new Rotation2d(angle));
        },
        turret);
  }
}
