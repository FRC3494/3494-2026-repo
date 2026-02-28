package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.QuadranglesUtil;
import java.util.function.Supplier;
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
 * mechanical limits). - TODO: Add logging/telemetry: distanceToTarget, angleToTarget, computed RPM,
 * hood/turret setpoints. - TODO: Wrap turret target to shortest rotation and consider motion
 * profiling / slew limiting to avoid commanding large instantaneous moves.
 *
 * <p>TODOs (structural) - TODO: Consider separating pure math (trajectory solver) into a utility
 * class so it can be tested independently from the Command and the hardware.
 */
public class AimShooterCommand extends Command {
  private Shooter shooter;

  private final Supplier<Pose2d> robotPose;

  // Persistent runtime offsets that operators can bump to trim aim during testing.
  // These are annotated for AutoLogOutput so they appear in logs/networktables for tuning.
  @Getter @Setter @AutoLogOutput private double flywheelOffsetRPM = 0.0;
  @Getter @Setter @AutoLogOutput private double hoodOffsetDeg = 0.0;
  @Getter @Setter @AutoLogOutput private double turretOffsetDeg = 0.0;

  // Flags that indicate whether the applied offset caused the final setpoint to be clipped by
  // safety clamps. These are updated every execute() and are useful to warn operators that
  // their trim pushed the requested setpoint outside allowed ranges.
  @Getter @AutoLogOutput private boolean flywheelOffsetClamped = false;
  @Getter @AutoLogOutput private boolean hoodOffsetClamped = false;
  @Getter @AutoLogOutput private boolean turretOffsetClamped = false;

  // Autologging: the target in field coordinates (translation only). This is visible via
  // AutoLogOutput and used by the aim math below. Keep this simple and documented so
  // other contributors can change the target at runtime.
  @Getter @Setter @AutoLogOutput
  Translation2d shooterTarget = QuadranglesUtil.toAllianceTranslation(hubLocation);

  public AimShooterCommand(Shooter shooter, Supplier<Pose2d> robotPose) {
    this.shooter = shooter;
    addRequirements(shooter);

    this.robotPose = robotPose;
  }

  @Override
  public void execute() {
    // Create robot Pose3d
    Translation2d robotLocation = robotPose.get().getTranslation();
    Rotation2d robotYaw = robotPose.get().getRotation();
    Pose3d robotPose3d =
        new Pose3d(
            robotLocation.getX(),
            robotLocation.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotYaw.getRadians()));

    // Create shooter Pose3d
    // move to constants later
    double shooterX = 0.0;
    double shooterY =
        Units.inchesToMeters(-2.074); // distance from robot center to shooter in meters
    double shooterZ = Units.inchesToMeters(13.72);

    Transform3d shooterTransform =
        new Transform3d(new Translation3d(shooterX, shooterY, shooterZ), new Rotation3d());
    Pose3d shooterPose3d = robotPose3d.transformBy(shooterTransform);

    // Create target Pose3d
    double targetHeight =
        Units.inchesToMeters(120.36); // height of the hub in meters, move to constants later
    Pose3d targetPose3d =
        new Pose3d(shooterTarget.getX(), shooterTarget.getY(), targetHeight, new Rotation3d());

    Translation3d translationToTarget = targetPose3d.minus(shooterPose3d).getTranslation();

    // Compute bearing to target in the XY plane.
    double angleRad = Math.atan2(translationToTarget.getY(), translationToTarget.getX());
    Rotation2d angleToTarget = Rotation2d.fromRadians(angleRad);

    // Convert world bearing into turret-relative angle by removing robot yaw
    Rotation2d turretAngle = angleToTarget.rotateBy(robotPose.get().getRotation().times(-1.0));

    // Implementation of desmos calculations provided in discord. Write-up with explainations in .md
    double maxHeight =
        calculateMaxHeight(shooterPose3d.getTranslation(), targetPose3d.getTranslation());
    double gravity = -9.81; // get from constants later
    double finalYVelocity =
        Math.sqrt(Math.abs(2 * gravity * (maxHeight - targetPose3d.getTranslation().getY())));
    double initialYVelocity =
        Math.sqrt(
            2 * gravity * (maxHeight - shooterPose3d.getTranslation().getY())
                + (maxHeight - targetPose3d.getTranslation().getY())
                + Math.pow(finalYVelocity, 2));
    double timeToTarget = (initialYVelocity - finalYVelocity) / gravity;
    double initialXVelocity = translationToTarget.getX() / timeToTarget;
    double flywheelRadius =
        Units.inchesToMeters(4) / 2.0; // get from constants later (assuming 3 inch diameter wheel)

    // Compute a raw target RPM from the physics model and apply persistent operator offset
    double rawCalculatedRPM =
        calculateFlywheelRPM(
            Math.sqrt(Math.pow(initialXVelocity, 2) + Math.pow(initialYVelocity, 2))
                * 60
                / (2 * Math.PI * flywheelRadius));

    // TODO: Send these limits to constants/config
    Double maxHoodAngle = 45.0;
    Double minHoodAngle = 24.2238027;

    Double maxTurretAngle = 360.0;
    Double minTurretAngle = 0.2005;

    Double maxFlywheelRPM = 5700.0; // to be tested and tuned
    Double minFlywheelRPM = 0.0;

  // Apply flywheel offset, clamp numerically, then convert to AngularVelocity
  double targetRPMBeforeClamp = rawCalculatedRPM + flywheelOffsetRPM;
  double targetRPM = MathUtil.clamp(targetRPMBeforeClamp, minFlywheelRPM, maxFlywheelRPM);
  // flag if the applied offset forced clamping
  flywheelOffsetClamped = Math.abs(targetRPM - targetRPMBeforeClamp) > 1e-6;
  AngularVelocity flywheelVelocity = RPM.of(targetRPM);

  // Compute hood angle (degrees), apply offset and clamp, then build Rotation2d correctly
  double hoodDegBeforeClamp = Math.toDegrees(Math.atan2(initialYVelocity, initialXVelocity)) + hoodOffsetDeg;
  double hoodDeg = MathUtil.clamp(hoodDegBeforeClamp, minHoodAngle, maxHoodAngle);
  hoodOffsetClamped = Math.abs(hoodDeg - hoodDegBeforeClamp) > 1e-6;
  Rotation2d hoodAngle = Rotation2d.fromDegrees(hoodDeg);

  // Apply turret offset to the previously computed turretAngle
  double turretDegBeforeClamp = turretAngle.getDegrees() + turretOffsetDeg;
  double turretDeg = MathUtil.clamp(turretDegBeforeClamp, minTurretAngle, maxTurretAngle);
  turretOffsetClamped = Math.abs(turretDeg - turretDegBeforeClamp) > 1e-6;
  turretAngle = Rotation2d.fromDegrees(turretDeg);

    // Send references to the shooter using the public API on Shooter. These delegate to
    // flywheel/hood/turret subsystem methods respectively.
    shooter.setFlywheelVelocity(flywheelVelocity);
    shooter.setHoodAngle(hoodAngle);
    shooter.setTurretPosition(turretAngle);
  }

  private double calculateFlywheelRPM(double velocity) {
    // Converts the calculated RPM (using physics-based model) to tested value
    // Requires testing of RPM vs Velocity to determine constants for conversion
    double conversionConstant = 1.0; // placeholder until testing is done
    return velocity * conversionConstant;
  }

  private double calculateMaxHeight(Translation3d currentLocation, Translation3d shooterTarget) {
    // Placeholder for max height calculation. Replace with actual implementation.
    return shooterTarget.getY() + 1.0; // example: 1 meter above the target
  }
    /**
   * Increment/decrement helpers for operator trimming during testing.
   *
   * <p>These modify the in-memory offsets (persist for the robot runtime). If you want them to
   * persist across reboots, consider storing/loading from Preferences or a JSON file.
   */
  public void bumpFlywheelOffsetRPM(double deltaRPM) {
    flywheelOffsetRPM += deltaRPM;
  }

  public void bumpHoodOffsetDeg(double deltaDeg) {
    hoodOffsetDeg += deltaDeg;
  }

  public void bumpTurretOffsetDeg(double deltaDeg) {
    turretOffsetDeg += deltaDeg;
  }

  public void resetAllOffsets() {
    flywheelOffsetRPM = 0.0;
    hoodOffsetDeg = 0.0;
    turretOffsetDeg = 0.0;
  }

}
