package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class SetTurretCommand extends Command {
  private final Turret turret;

  private final Supplier<Rotation2d> turretAngle;

  public SetTurretCommand(Turret turret, Supplier<Rotation2d> turretAngle) {
    this.turret = turret;
    addRequirements(turret);

    this.turretAngle = turretAngle;
  }

  @Override
  public void execute() {
    turret.setPosition(turretAngle.get().getRotations());
  }
}
