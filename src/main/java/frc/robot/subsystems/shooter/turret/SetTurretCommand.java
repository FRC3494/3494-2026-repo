package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SetTurretCommand extends Command {
  private final Turret turret;

  private final DoubleSupplier turretAngleRot;
  private final Supplier<Voltage> arbFFVolts;

  public SetTurretCommand(
      Turret turret, DoubleSupplier turretAngleRot, Supplier<Voltage> arbFFVolts) {
    this.turret = turret;
    addRequirements(turret);

    this.turretAngleRot = turretAngleRot;
    this.arbFFVolts = arbFFVolts;
  }

  @Override
  public void execute() {
    turret.setTurretArbFF(arbFFVolts.get());
    turret.setPosition(turretAngleRot.getAsDouble());
  }
}
