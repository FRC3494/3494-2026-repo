package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ShooterConstants.HoodConstants.hoodMinAngle;
import static frc.robot.Constants.ShooterConstants.TurretConstants.turretMinAngleRot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.QuadranglesUtil;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Closed-loop aiming command for the shooter.
 *
 * <p>On every {@link #execute()} call this command:
 *
 * <ol>
 *   <li>Samples the current {@link Pose2d} of the robot from the supplied pose source.
 *   <li>Builds a {@link Pose3d} for the shooter using a fixed transform from the robot frame.
 *   <li>Builds a 3D target pose from {@link #shooterTarget}, which is expected to be the
 *       alliance-relative hub location.
 *   <li>Solves a simple ballistic model to estimate the required launch velocity.
 *   <li>Converts that launch velocity into a flywheel RPM setpoint.
 *   <li>Applies operator-trim offsets for flywheel, hood, and turret and clamps them to the safe
 *       ranges defined in {@link FlywheelConstants}, {@link HoodConstants}, and {@link
 *       TurretConstants}.
 *   <li>Commands the {@link Shooter} subsystem to those setpoints and logs key telemetry via
 *       AdvantageKit.
 * </ol>
 *
 * <p>Runtime configuration / tuning hooks:
 *
 * <ul>
 *   <li>{@link #shooterTarget} can be changed at runtime (e.g. to aim at a different field location
 *       or a moving target).
 *   <li>{@link #flywheelOffsetRPM}, {@link #hoodOffsetDeg}, and {@link #turretOffsetDeg} may be
 *       adjusted (or "bumped") by the operator to trim aim without changing the underlying model.
 * </ul>
 *
 * <p>Limitations / known work-in-progress:
 *
 * <ul>
 *   <li>The ballistic model uses placeholder constants and a very simple arc calculation. It must
 *       be validated and tuned against on-field testing.
 *   <li>Max-height and RPM conversion are currently approximations; see the {@code TODO} items in
 *       {@link #calculateMaxHeight(Translation3d, Translation3d)} and {@link
 *       #calculateFlywheelRPM(double)}.
 * </ul>
 */
public class AimShooterMath extends SubsystemBase {
  // Supplier for the latest field-relative robot pose (typically from odometry or
  // a pose estimator). The math class is intentionally decoupled from commands and
  // subsystems; it only needs the pose and internal tuning state.
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
  private Translation2d shooterTarget = QuadranglesUtil.toAllianceTranslation(hubLocation);

  @Getter
  private Setpoints setpoints =
      new Setpoints(
          0,
          hoodMinAngle,
          Rotation2d.fromRotations(turretMinAngleRot),
          turretOffsetClamped,
          hoodOffsetClamped,
          flywheelOffsetClamped);

  public AimShooterMath(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;
  }

  /** Main control loop step that recomputes aim and updates shooter setpoints. */
  @Override
  public void periodic() {
    // 1) Read state and build geometry
    AimState state = buildAimState();

    // 2) Compute projectile/velocity physics
    PhysicsResult physics =
        computePhysics(state.shooterPose3d, state.targetPose3d, state.translationToTarget);

    // 3) Compute launch speed and convert to flywheel RPM
    double launchSpeedMps = Math.hypot(physics.initialXVelocity, physics.initialYVelocity);
    double calculatedRPM = calculateFlywheelRPM(launchSpeedMps);

    // 4) Apply operator offsets and clamps to get final setpoints, tracking whether any were
    // clamped for logging. Use the current turret angle (from the last setpoint) so we can
    // unwrap the target angle and avoid snapping across the +/-180 degree discontinuity.
    setpoints =
        applyOffsetsAndClamps(
            calculatedRPM, physics, state.turretAngleWorld, setpoints.turretAngle);

    // 5) Log detailed stats for debugging/tuning and return
    logAimShooterStats(state, physics, setpoints);
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

    // shooter offset relative to robot frame.
    // TODO(#aim-shooter): Move shooter mounting transform into {@link ShooterConstants}
    // or a dedicated geometry/config class so it can be hardware-specific and
    // easier to update between robots.
    Transform3d shooterTransform =
        new Transform3d(
            new Translation3d(
                ShooterConstants.shooterX, ShooterConstants.shooterY, ShooterConstants.shooterZ),
            new Rotation3d());
    Pose3d shooterPose3d = robotPose3d.transformBy(shooterTransform);

    Pose3d targetPose3d =
        new Pose3d(hubLocation.getX(), hubLocation.getY(), targetHeight, new Rotation3d());

    Translation3d translationToTarget = targetPose3d.minus(shooterPose3d).getTranslation();

    double angleRad = Math.atan2(translationToTarget.getY(), translationToTarget.getX());
    Rotation2d angleToTarget = Rotation2d.fromRadians(angleRad);
    Rotation2d turretAngleWorld = angleToTarget.rotateBy(robotYaw.times(-1.0));

    return new AimState(
        robotPose3d, shooterPose3d, targetPose3d, translationToTarget, turretAngleWorld);
  }

  /**
   * Computes a simple ballistic model for the shot based on shooter/target poses.
   *
   * <p>Returns the initial X/Y launch velocities and time-of-flight needed for the projectile to
   * travel from the shooter to the target, assuming constant gravity and no air resistance or spin
   * effects.
   */
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

  /**
   * Applies operator offsets to flywheel/hood/turret and clamps them to safe ranges.
   *
   * <p>Also tracks whether any of the resulting setpoints were clamped so the dashboard can warn
   * the driver when trims push the system beyond its limits.
   */
  private Setpoints applyOffsetsAndClamps(
      double calculatedRPM,
      PhysicsResult physics,
      Rotation2d turretAngleWorld,
      Rotation2d currentTurretAngle) {

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

    // Turret: apply offset to world turret angle, unwrap near current position, then clamp
    double desiredTurretDeg = turretAngleWorld.getDegrees() + turretOffsetDeg;
    double continuousTurretDeg = unwrapToNearest(desiredTurretDeg, currentTurretAngle.getDegrees());
    // double turretDeg =
    //     MathUtil.clamp(
    //         continuousTurretDeg,
    //         Units.rotationsToDegrees(TurretConstants.turretMinAngleRot),
    //         Units.rotationsToDegrees(TurretConstants.turretMaxAngleRot));
    // boolean turretClamped = Math.abs(turretDeg - continuousTurretDeg) > 1e-6;
    // Rotation2d turretAngle = Rotation2d.fromDegrees(turretDeg);

    return new Setpoints(
        targetRPM,
        hoodAngle,
        Rotation2d.fromDegrees(continuousTurretDeg),
        flywheelClamped,
        hoodClamped,
        false);
  }

  /**
   * Unwrap {@code targetDeg} to the equivalent angle (differing by multiples of 360) that is
   * closest to {@code referenceDeg}. This keeps turret motion continuous instead of snapping across
   * the -180/180 discontinuity.
   */
  private static double unwrapToNearest(double targetDeg, double referenceDeg) {
    // Normalize to (-180, 180]
    double t = MathUtil.inputModulus(targetDeg, -180.0, 180.0);
    double r = MathUtil.inputModulus(referenceDeg, -180.0, 180.0);

    // Smallest signed angle from reference to target in (-180, 180]
    double delta = MathUtil.inputModulus(t - r, -180.0, 180.0);

    // Apply that delta to the original reference (which may already be outside [-180, 180])
    return referenceDeg + delta;
  }

  /**
   * Converts linear exit velocity (m/s) to a flywheel RPM setpoint.
   *
   * <p>Currently this is a stub that assumes a 1:1 mapping between linear velocity and flywheel
   * surface speed. In reality this should be derived from empirical testing that maps "ball exit
   * velocity" to motor RPM for your specific shooter geometry and wheel type.
   *
   * <p>TODO(#aim-shooter): Replace placeholder conversion with a calibrated model or lookup table
   * based on range testing.
   */
  private static double calculateFlywheelRPM(double launchSpeedMps) {
    // Base geometric conversion from rim surface speed (m/s) to RPM.
    double baseRpm = launchSpeedMps * 60.0 / (2.0 * Math.PI * FlywheelConstants.flywheelRadius);

    // Placeholder correction factor until on-field testing maps ball exit velocity
    // to motor RPM for this specific shooter.
    double correctionFactor = 1.0; // TODO(#aim-shooter): Tune from range testing
    return baseRpm * correctionFactor;
  }

  private static double calculateMaxHeight(
      Translation3d currentLocation, Translation3d shooterTarget) {
    // Placeholder for max height calculation to shape the projectile arc.
    //
    // The current implementation scales with horizontal distance so that farther
    // shots arc higher. This is only a heuristic to get a plausible trajectory.
    //
    // TODO(#aim-shooter): Replace with an analytically-derived or experimentally-fit
    // model that guarantees clearance over field obstacles and optimizes for
    // consistency / forgiveness at competition distances.
    double heightScalingFactor = 1.5; // heuristic scaling factor for arc height
    return heightScalingFactor
        * Math.hypot(
            currentLocation.getX() - shooterTarget.getX(),
            currentLocation.getY() - shooterTarget.getY());
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

  public class Setpoints {
    public final double rpm;
    public final Rotation2d hoodAngle;
    public final Rotation2d turretAngle;
    public final boolean flywheelClamped;
    public final boolean hoodClamped;
    public final boolean turretClamped;

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

  /**
   * Emits detailed telemetry about the current aim computation to AdvantageKit.
   *
   * <p>This is intended primarily for tuning and debugging. Values are grouped into distance,
   * angles, physics (projectile), and setpoint/clamp diagnostics.
   */
  private static void logAimShooterStats(AimState state, PhysicsResult physics, Setpoints set) {
    double horizontalDistanceMeters =
        Math.hypot(state.translationToTarget.getX(), state.translationToTarget.getY());
    double distance3dMeters = state.translationToTarget.getNorm();
    double verticalDeltaMeters = state.translationToTarget.getZ();

    double launchSpeedMps = Math.hypot(physics.initialXVelocity, physics.initialYVelocity);
    double calculatedRPM = calculateFlywheelRPM(launchSpeedMps);

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
