package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Common interface for shooter aiming math implementations so they can be swapped transparently
 * (e.g. ballistic vs. interpolation-based models).
 */
public interface ShooterAimModel extends Subsystem {
  /** Returns the current turret angle setpoint in rotations. */
  double getTurretAngleRot();

  /** Returns the current turret feedforward voltage. */
  Voltage getTurretFF();

  /** Returns the current hood angle setpoint. */
  Rotation2d getHoodAngle();

  /** Returns the current flywheel speed setpoint. */
  AngularVelocity getFlywheelSpeed();

  boolean isInAllianceZone();

  // ===== Optional trim application helpers =====

  /**
   * Applies any model-specific trim to a base flywheel speed.
   *
   * <p>Default implementation returns the base speed unchanged; models that support trim can
   * override this.
   */
  default AngularVelocity applyFlywheelTrim(AngularVelocity baseSpeed) {
    return baseSpeed;
  }

  /**
   * Applies any model-specific trim to a base hood angle.
   *
   * <p>Default implementation returns the base angle unchanged; models that support trim can
   * override this.
   */
  default Rotation2d applyHoodTrim(Rotation2d baseAngle) {
    return baseAngle;
  }

  // ===== Optional runtime trim state helpers =====

  // Default implementations are no-ops so non-trim-capable models don't need to care.

  default double getTurretTrimRot() {
    return 0.0;
  }

  default void setTurretTrim(double trimRot) {}

  default Rotation2d getHoodTrim() {
    return Rotation2d.kZero;
  }

  default void setHoodTrim(Rotation2d trim) {}

  default AngularVelocity getFlywheelTrim() {
    return RPM.of(0.0);
  }

  default void setFlywheelTrim(AngularVelocity trim) {}

  default Distance getDistanceTrim() {
    return Inches.of(0.0);
  }

  default void setDistanceTrim(Distance trim) {}

  default Distance getXTrim() {
    return Inches.of(0.0);
  }

  default void setXTrim(Distance trim) {}

  default Distance getYTrim() {
    return Inches.of(0.0);
  }

  default void setYTrim(Distance trim) {}
}
