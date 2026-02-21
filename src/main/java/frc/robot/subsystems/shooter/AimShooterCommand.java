package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.QuadranglesUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * AimShooterCommand
 *
 * <p>Current responsibility: - Compute a target bearing from the robot's current pose to a fixed
 * shooter target (stored in {@code shooterTarget}). - Convert that world bearing into a
 * turret-relative Rotation2d and call {@link Shooter#setPosition(AngularVelocity, Rotation2d,
 * Rotation2d)} with placeholder flywheel/hood setpoints.
 *
 * <p>Notes about the existing implementation (according to AI): - This class extends the
 * (deprecated/old) {@code Command} interface; it currently only implements {@code execute()} and
 * therefore behaves like a continuously-running action when scheduled. Consider converting to
 * {@link edu.wpi.first.wpilibj2.command.CommandBase} or implementing
 * initialize()/end()/isFinished() for a clearer lifecycle.
 *
 * <p>TODOs (high priority) - TODO: Implement a mapping from distance -> flywheel RPM (e.g. lookup
 * table or physics model). - TODO: Implement a mapping from distance -> hood angle (and clamp to
 * mechanical limits). - TODO: Add validation / safety checks: null checks for pose/rotation,
 * NaN/Inf guards, and clamp RPM/angles to safe values. - TODO: Add logging/telemetry:
 * distanceToTarget, angleToTarget, computed RPM, hood/turret setpoints. - TODO: Wrap turret target
 * to shortest rotation and consider motion profiling / slew limiting to avoid commanding large
 * instantaneous moves.
 *
 * <p>TODOs (structural) - TODO: Consider separating pure math (trajectory solver) into a utility
 * class so it can be tested independently from the Command and the hardware.
 */
public class AimShooterCommand extends Command {
  private Shooter shooter;
  private Drive drive;

  // Autologging: the target in field coordinates (translation only). This is visible via
  // AutoLogOutput and used by the aim math below. Keep this simple and documented so
  // other contributors can change the target at runtime.
  @Getter @Setter @AutoLogOutput
  Translation2d shooterTarget = QuadranglesUtil.toAllianceTranslation(hubLocation);

  public AimShooterCommand(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    addRequirements(shooter);

    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // TODO: Consider making drive.getPose() return an Optional or add a helper getter that
    Translation2d currentLocation = drive.getPose().getTranslation();

    // Vector from robot to target in field coordinates
    Translation2d translationToTarget = shooterTarget.minus(currentLocation);
    // Distance to target (meters) is available when needed via:
    // double distanceToTarget = currentLocation.getDistance(shooterTarget);

    // Compute bearing to target in the XY plane.
    double angleRad = Math.atan2(translationToTarget.getY(), translationToTarget.getX());
    Rotation2d angleToTarget = Rotation2d.fromRadians(angleRad);

    // Convert world bearing into turret-relative angle by removing robot yaw
    Rotation2d turretAngle = angleToTarget.rotateBy(drive.getRotation().times(-1.0));

    // Implementation of desmos calculations provided in discord. Write-up with explainations in .md
    translationToTarget = shooterTarget.minus(currentLocation);
    double maxHeight = calculateMaxHeight(currentLocation, shooterTarget);
    double gravity = 9.81; // get from constants later
    double finalYVelocity = Math.sqrt(Math.abs(2 * gravity * (maxHeight - shooterTarget.getY())));
    double initialYVelocity =
        Math.sqrt(
            2 * gravity * (maxHeight - currentLocation.getY())
                + (maxHeight - shooterTarget.getY())
                + Math.pow(finalYVelocity, 2));
    double timeToTarget = (initialYVelocity - finalYVelocity) / gravity;
    double initialXVelocity = translationToTarget.getX() / timeToTarget;
    double flywheelRadius = 0.0762; // get from constants later (assuming 3 inch diameter wheel)
    double calculatedRPM =
        calculateFlywheelRPM(
            Math.sqrt(Math.pow(initialXVelocity, 2) + Math.pow(initialYVelocity, 2))
                * 60
                / (2 * Math.PI * flywheelRadius));

    AngularVelocity flywheelVelocity = RPM.of(calculatedRPM);
    // add feedforward/closed-loop params in the Shooter subsystem rather than here.

    Rotation2d hoodAngle = Rotation2d.fromRadians(Math.atan2(initialYVelocity, initialXVelocity));

    // TODO: Send to constants:
    Double maxHoodAngle = 45.0;
    Double minHoodAngle = 24.2238027;

    Double maxTurretAngle = 360.0;
    Double minTurretAngle = 0.2005;

    Double maxFlywheelRPM = 5700.0; // to be tested and tuned
    Double minFlywheelRPM = 0.0;

    hoodAngle = new Rotation2d(MathUtil.clamp(hoodAngle.getDegrees(), minHoodAngle, maxHoodAngle));
    turretAngle =
        new Rotation2d(MathUtil.clamp(turretAngle.getDegrees(), minTurretAngle, maxTurretAngle));
    flywheelVelocity =
        RPM.of(MathUtil.clamp(flywheelVelocity.in(RPM), minFlywheelRPM, maxFlywheelRPM));
    // Send references to the shooter. This method should accept units documented in Shooter.
    shooter.setPosition(flywheelVelocity, hoodAngle, turretAngle);
  }

  private double calculateFlywheelRPM(double calculatedRPM) {
    // Converts the calculated RPM (using physics-based model) to tested value
    // Requires testing of RPM vs Velocity to determine constants for conversion
    double conversionConstant = 1.0; // placeholder until testing is done
    return calculatedRPM * conversionConstant;
  }

  private double calculateMaxHeight(Translation2d currentLocation, Translation2d shooterTarget) {
    // Placeholder for max height calculation. Replace with actual implementation.
    return shooterTarget.getY() + 1.0; // example: 1 meter above the target
  }
}
