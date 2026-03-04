package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class SetTurretCommand extends Command {
  private final Turret turret;

  private final DoubleSupplier turretAngleRot;

  public SetTurretCommand(Turret turret, DoubleSupplier turretAngleRot) {
    this.turret = turret;
    addRequirements(turret);

    this.turretAngleRot = turretAngleRot;
  }

  @Override
  public void execute() {
    turret.setPosition(turretAngleRot.getAsDouble());
  }
}
