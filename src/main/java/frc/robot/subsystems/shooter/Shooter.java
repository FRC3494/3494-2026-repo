package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

public class Shooter extends SubsystemBase {
  private Flywheel flywheel;
  private Hood hood;
  private Turret turret;

  public Shooter() {
    flywheel = new Flywheel();
    hood = new Hood();
    turret = new Turret();
  }

  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheel.setVelocity(velocity);
  }

  public void setHoodAngle(Rotation2d angle) {
    hood.setPosition(angle);
  }

  public void setTurretPosition(Rotation2d position) {
    turret.setPosition(position);
  }

  public AngularVelocity getFlywheelSetpoint() {
    return flywheel.getFlywheelSetpoint();
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheel.getVelocity();
  }

  public Rotation2d getHoodSetpoint() {
    return hood.getHoodSetpoint();
  }

  public Rotation2d getTurretSetpoint() {
    return turret.getTurretSetpoint();
  }

  public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return flywheel.sysIdQuasistatic(direction);
  }

  public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return flywheel.sysIdDynamic(direction);
  }
}
