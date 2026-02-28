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
import frc.robot.Constants.ShooterConstants.FlywheelConstants;
import frc.robot.Constants.ShooterConstants.HoodConstants;
import frc.robot.Constants.ShooterConstants.TurretConstants;
import frc.robot.util.QuadranglesUtil;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
    // 1) Read state and build geometry
    AimState state = buildAimState();

    // 2) Compute projectile/velocity physics
    PhysicsResult physics =
        computePhysics(state.shooterPose3d, state.targetPose3d, state.translationToTarget);

    // 3) Compute raw RPM from physics
    double linearVelocity = Math.hypot(physics.initialXVelocity, physics.initialYVelocity);
    double calculatedRPM =
        calculateFlywheelRPM(
            linearVelocity * 60.0 / (2.0 * Math.PI * FlywheelConstants.flywheelRadius));

    // 4) Apply offsets and clamp to limits, update clamped flags
    Setpoints set = applyOffsetsAndClamps(calculatedRPM, physics, state.turretAngleWorld);

    // 5) Send setpoints to hardware
    flywheelOffsetClamped = set.flywheelClamped;
    hoodOffsetClamped = set.hoodClamped;
    turretOffsetClamped = set.turretClamped;

    shooter.setFlywheelVelocity(RPM.of(set.rpm));
    shooter.setHoodAngle(set.hoodAngle);
    shooter.setTurretPosition(set.turretAngle);

    // 6) (Optional) Log additional info for debugging/tuning
    logAimShooterStats(state, physics, set);
  }

  private AimState buildAimState() {
    Pose2d robot2d = robotPose.get();
    Translation2d robotLocation = robot2d.getTranslation();
    Rotation2d robotYaw = robot2d.getRotation();

    Pose3d robotPose3d =
        new Pose3d(
            robotLocation.getX(),
            robotLocation.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotYaw.getRadians()));

    // shooter offset relative to robot frame (move to constants later)
    double shooterX = 0.0;
    double shooterY = Units.inchesToMeters(-2.074);
    double shooterZ = Units.inchesToMeters(13.72);
    Transform3d shooterTransform =
        new Transform3d(new Translation3d(shooterX, shooterY, shooterZ), new Rotation3d());
    Pose3d shooterPose3d = robotPose3d.transformBy(shooterTransform);

    double targetHeight = Units.inchesToMeters(120.36); // hub height
    Pose3d targetPose3d =
        new Pose3d(shooterTarget.getX(), shooterTarget.getY(), targetHeight, new Rotation3d());

    Translation3d translationToTarget = targetPose3d.minus(shooterPose3d).getTranslation();

    double angleRad = Math.atan2(translationToTarget.getY(), translationToTarget.getX());
    Rotation2d angleToTarget = Rotation2d.fromRadians(angleRad);
    Rotation2d turretAngleWorld = angleToTarget.rotateBy(robotYaw.times(-1.0));

    return new AimState(
        robotPose3d, shooterPose3d, targetPose3d, translationToTarget, turretAngleWorld);
  }

  private static PhysicsResult computePhysics(
      Pose3d shooterPose3d, Pose3d targetPose3d, Translation3d translationToTarget) {
    double maxHeight =
        calculateMaxHeight(shooterPose3d.getTranslation(), targetPose3d.getTranslation());

    double finalYVelocity =
        Math.sqrt(Math.abs(2 * gravity * (maxHeight - targetPose3d.getTranslation().getY())));
    double initialYVelocity =
        Math.sqrt(
            2 * gravity * (maxHeight - shooterPose3d.getTranslation().getY())
                + (maxHeight - targetPose3d.getTranslation().getY())
                + Math.pow(finalYVelocity, 2));
    double timeToTarget = (initialYVelocity - finalYVelocity) / gravity;
    double initialXVelocity = translationToTarget.getX() / timeToTarget;

    return new PhysicsResult(initialXVelocity, initialYVelocity, timeToTarget);
  }

  private Setpoints applyOffsetsAndClamps(
      double calculatedRPM, PhysicsResult physics, Rotation2d turretAngleWorld) {

    // Flywheel: apply offset then clamp
    double targetRPMBeforeClamp = calculatedRPM + flywheelOffsetRPM;
    double targetRPM =
        MathUtil.clamp(
            targetRPMBeforeClamp,
            FlywheelConstants.flywheelMinSpeed.in(RPM),
            FlywheelConstants.flywheelMaxSpeed.in(RPM));
    boolean flywheelClamped = Math.abs(targetRPM - targetRPMBeforeClamp) > 1e-6;

    // Hood angle: compute from ballistic angles, then offset/clamp
    double hoodDegBeforeClamp =
        Math.toDegrees(Math.atan2(physics.initialYVelocity, physics.initialXVelocity))
            + hoodOffsetDeg;
    double hoodDeg =
        MathUtil.clamp(
            hoodDegBeforeClamp,
            HoodConstants.hoodMinAngle.getDegrees(),
            HoodConstants.hoodMaxAngle.getDegrees());
    boolean hoodClamped = Math.abs(hoodDeg - hoodDegBeforeClamp) > 1e-6;
    Rotation2d hoodAngle = Rotation2d.fromDegrees(hoodDeg);

    // Turret: apply offset to world turret angle then clamp
    double turretDegBeforeClamp = turretAngleWorld.getDegrees() + turretOffsetDeg;
    double turretDeg =
        MathUtil.clamp(
            turretDegBeforeClamp,
            TurretConstants.turretMinAngle.getDegrees(),
            TurretConstants.turretMaxAngle.getDegrees());
    boolean turretClamped = Math.abs(turretDeg - turretDegBeforeClamp) > 1e-6;
    Rotation2d turretAngle = Rotation2d.fromDegrees(turretDeg);

    return new Setpoints(
        targetRPM, hoodAngle, turretAngle, flywheelClamped, hoodClamped, turretClamped);
  }

  private static double calculateFlywheelRPM(double velocity) {
    // Converts the calculated RPM (using physics-based model) to tested value
    // Requires testing of RPM vs Velocity to determine constants for conversion
    double conversionConstant = 1.0; // placeholder until testing is done
    return velocity * conversionConstant;
  }

  private static double calculateMaxHeight(
      Translation3d currentLocation, Translation3d shooterTarget) {
    // Placeholder for max height calculation. Replace with actual implementation.
    double heightScalingFactor = 1.5; // example scaling factor to ensure the arc clears the target
    return heightScalingFactor
        * Math.hypot(
            currentLocation.getX() - shooterTarget.getX(),
            currentLocation.getY() - shooterTarget.getY()); // example: 1 meter above the target
  }

  // Small helper structs to make execute() readable and testable
  private static class AimState {
    final Pose3d robotPose3d;
    final Pose3d shooterPose3d;
    final Pose3d targetPose3d;
    final Translation3d translationToTarget;
    final Rotation2d turretAngleWorld;

    AimState(
        Pose3d robotPose3d,
        Pose3d shooterPose3d,
        Pose3d targetPose3d,
        Translation3d translationToTarget,
        Rotation2d turretAngleWorld) {
      this.robotPose3d = robotPose3d;
      this.shooterPose3d = shooterPose3d;
      this.targetPose3d = targetPose3d;
      this.translationToTarget = translationToTarget;
      this.turretAngleWorld = turretAngleWorld;
    }
  }

  private static class PhysicsResult {
    final double initialXVelocity;
    final double initialYVelocity;
    final double timeToTarget;

    PhysicsResult(double initialXVelocity, double initialYVelocity, double timeToTarget) {
      this.initialXVelocity = initialXVelocity;
      this.initialYVelocity = initialYVelocity;
      this.timeToTarget = timeToTarget;
    }
  }

  private static class Setpoints {
    final double rpm;
    final Rotation2d hoodAngle;
    final Rotation2d turretAngle;
    final boolean flywheelClamped;
    final boolean hoodClamped;
    final boolean turretClamped;

    Setpoints(
        double rpm,
        Rotation2d hoodAngle,
        Rotation2d turretAngle,
        boolean flywheelClamped,
        boolean hoodClamped,
        boolean turretClamped) {
      this.rpm = rpm;
      this.hoodAngle = hoodAngle;
      this.turretAngle = turretAngle;
      this.flywheelClamped = flywheelClamped;
      this.hoodClamped = hoodClamped;
      this.turretClamped = turretClamped;
    }
  }

  /** Increment/decrement helpers for operator trimming during testing. */
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

  private static void logAimShooterStats(AimState state, PhysicsResult physics, Setpoints set) {
    double horizontalDistanceMeters =
        Math.hypot(state.translationToTarget.getX(), state.translationToTarget.getY());
    double distance3dMeters = state.translationToTarget.getNorm();
    double verticalDeltaMeters = state.translationToTarget.getZ();

    double launchSpeedMps = Math.hypot(physics.initialXVelocity, physics.initialYVelocity);
    double calculatedRPM =
        calculateFlywheelRPM(
            launchSpeedMps * 60.0 / (2.0 * Math.PI * FlywheelConstants.flywheelRadius));

    Logger.recordOutput("AimShooter/Distance/HorizontalMeters", horizontalDistanceMeters);
    Logger.recordOutput("AimShooter/Distance/Distance3dMeters", distance3dMeters);
    Logger.recordOutput("AimShooter/Distance/VerticalDeltaMeters", verticalDeltaMeters);

    Logger.recordOutput("AimShooter/Angles/TurretWorldDeg", state.turretAngleWorld.getDegrees());
    Logger.recordOutput("AimShooter/Angles/HoodSetpointDeg", set.hoodAngle.getDegrees());
    Logger.recordOutput("AimShooter/Angles/TurretSetpointDeg", set.turretAngle.getDegrees());

    Logger.recordOutput("AimShooter/Physics/InitialXVelocityMps", physics.initialXVelocity);
    Logger.recordOutput("AimShooter/Physics/InitialYVelocityMps", physics.initialYVelocity);
    Logger.recordOutput("AimShooter/Physics/LaunchSpeedMps", launchSpeedMps);
    Logger.recordOutput("AimShooter/Physics/TimeToTargetSec", physics.timeToTarget);
    Logger.recordOutput("AimShooter/Physics/CalculatedRPM", calculatedRPM);

    Logger.recordOutput("AimShooter/Setpoints/RPM", set.rpm);

    Logger.recordOutput("AimShooter/Clamp/FlywheelClamped", set.flywheelClamped);
    Logger.recordOutput("AimShooter/Clamp/HoodClamped", set.hoodClamped);
    Logger.recordOutput("AimShooter/Clamp/TurretClamped", set.turretClamped);
  }
}
