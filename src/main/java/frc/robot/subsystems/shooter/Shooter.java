package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hopper.Hopper;
import frc.robot.subsystems.shooter.turret.Turret;

public class Shooter extends SubsystemBase {
  private Flywheel flywheel;
  private Hood hood;
  private Hopper hopper;
  private Turret turret;

  public Shooter() {
    flywheel = new Flywheel();
    hood = new Hood();
    hopper = new Hopper();
    turret = new Turret();
  }

  public void setPosition(
      AngularVelocity flywheelVelocity, Rotation2d hoodPosition, Rotation2d turretPosition) {
    flywheel.setVelocity(flywheelVelocity);
    hood.setPosition(hoodPosition);
    turret.setPosition(turretPosition);
  }
}
