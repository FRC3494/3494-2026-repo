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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap.Shooter;
import frc.robot.Constants.ShooterConstants;
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
 * Closed-loop ballistic aiming model for the shooter.
 *
 * <p>On every {@link #execute()} call this command:
 *
 * <ol>
 *   <li>Samples the current {@link Pose2d} of the robot from the supplied pose source.
 *   <li>Builds a {@link Pose3d} for the shooter using a fixed transform from the robot frame.
 *   <li>Selects a 2D field target using the same zone logic as {@link AimShooterMathLinear}:
 *       alliance-zone shots aim at the alliance hub (with X/Y trim), otherwise the model picks the
 *       nearest neutral-zone shooting target.
 *   <li>Builds a 3D target pose from that selected target.
 *   <li>Solves a simple ballistic model to estimate the required launch velocity.
 *   <li>Converts that launch velocity into a flywheel RPM setpoint.
 *   <li>Applies operator trims:
 *       <ul>
 *         <li>target-space trims: X trim, Y trim, and distance trim
 *         <li>setpoint trims: flywheel RPM, hood angle, and turret angle offsets
 *       </ul>
 *   <li>Clamps the resulting flywheel and hood setpoints to the safe ranges defined in {@link
 *       FlywheelConstants}, {@link HoodConstants}, and {@link TurretConstants}.
 *   <li>Commands the {@link Shooter} subsystem to those setpoints and logs key telemetry via
 *       AdvantageKit.
 * </ol>
 *
 * <p>Runtime configuration / tuning hooks:
 *
 * <ul>
 *   <li>{@link #shooterTarget} remains available as a configurable target concept, but the current
 *       implementation selects between alliance-zone and neutral-zone targets internally.
 *   <li>{@link #flywheelOffsetRPM}, {@link #hoodOffsetDeg}, and {@link #turretOffsetDeg} adjust the
 *       final commanded setpoints.
 *   <li>{@link #distanceTrimInches}, {@link #xTrimInches}, and {@link #yTrimInches} adjust the
 *       selected target before or during the ballistic solve.
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
 *   <li>Distance trim is applied by shifting the motion-compensated target along the shooter-to-
 *       target ray, which is a practical geometric approximation rather than a full re-solve of a
 *       richer guidance model.
 * </ul>
 */
public class AimShooterMathKinematics extends SubsystemBase implements ShooterAimModel {
  // Supplier for the latest field-relative robot pose (typically from odometry or
  // a pose estimator). The math class is intentionally decoupled from commands and
  // subsystems; it only needs the pose and internal tuning state.
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  // Persistent runtime offsets that operators can bump to trim aim during testing.
  // These are annotated for AutoLogOutput so they appear in logs/networktables for tuning.
  @Getter @Setter @AutoLogOutput private double flywheelOffsetRPM = 0.0;
  @Getter @Setter @AutoLogOutput private double hoodOffsetDeg = 0.0;
  @Getter @Setter @AutoLogOutput private double turretOffsetDeg = 0.0;
  @Getter @Setter @AutoLogOutput private double distanceTrimInches = 0.0;
  @Getter @Setter @AutoLogOutput private double xTrimInches = 0.0;
  @Getter @Setter @AutoLogOutput private double yTrimInches = 0.0;
  @AutoLogOutput private Voltage turretFF = Volts.of(0.0);

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

  private double lastLoopTimestamp = Timer.getTimestamp();
  private double previousTOF = 0.0;
  private Translation2d previousRobotSpeed = new Translation2d();

  public AimShooterMathKinematics(
      Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> chassisSpeeds) {
    this.robotPoseSupplier = robotPose;
    this.chassisSpeedsSupplier = chassisSpeeds;
  }

  /** Main control loop step that recomputes aim and updates shooter setpoints. */
  @Override
  public void periodic() {
    Pose2d currentRobotPose = robotPoseSupplier.get();
    ChassisSpeeds currentSpeeds = chassisSpeedsSupplier.get();

    // 1) Read state and build geometry
    AimState state = buildAimState(currentRobotPose, currentSpeeds);

    // 2) Compute projectile/velocity physics
    PhysicsResult physics = computePhysics(state);

    // 3) Compute launch speed and convert to flywheel RPM
    double launchSpeedMps = Math.hypot(physics.initialXVelocity, physics.initialYVelocity);
    double calculatedRPM = calculateFlywheelRPM(launchSpeedMps);

    // 4) Apply operator offsets and clamps to get final setpoints, tracking whether any were
    // clamped for logging. Use the current turret angle (from the last setpoint) so we can
    // unwrap the target angle and avoid snapping across the +/-180 degree discontinuity.
    setpoints =
        applyOffsetsAndClamps(
            calculatedRPM,
            physics,
            state.turretAngleWorld,
            setpoints.turretAngle,
            flywheelOffsetRPM,
            hoodOffsetDeg,
            turretOffsetDeg);

    turretFF = getTurretFF(state, physics.timeToTarget);

    // 5) Log detailed stats for debugging/tuning and return
    logAimShooterStats(state, physics, setpoints);
  }

  /**
   * Builds the geometric state used by the ballistic aim calculation.
   *
   * <p>This method:
   *
   * <ol>
   *   <li>Creates 3D robot and shooter poses from the current robot pose.
   *   <li>Selects the target using alliance-zone vs neutral-zone logic.
   *   <li>Applies X/Y trim directly to alliance-zone hub targeting.
   *   <li>Estimates a first-pass time of flight for motion compensation.
   *   <li>Applies distance trim by moving the motion-compensated target along the current line of
   *       fire.
   * </ol>
   */
  private AimState buildAimState(Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    Translation2d robotLocation = robotPose.getTranslation();
    Rotation2d robotYaw = robotPose.getRotation();

    // Current chassis speeds (field-relative)
    ChassisSpeeds speeds = chassisSpeeds;

    Pose3d robotPose3d =
        new Pose3d(
            robotLocation.getX(),
            robotLocation.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotYaw.getRadians()));

    // The Shooter's position relative to the robot
    Transform3d shooterTransform =
        new Transform3d(
            new Translation3d(
                ShooterConstants.shooterX, ShooterConstants.shooterY, ShooterConstants.shooterZ),
            new Rotation3d());
    // The Shooter's position in field coordinates
    Pose3d shooterPose3d = robotPose3d.transformBy(shooterTransform);

    Translation2d shooterTranslation = shooterPose3d.getTranslation().toTranslation2d();
    Translation2d allianceHubLocation = QuadranglesUtil.toAllianceTranslation(hubLocation);
    boolean inAllianceZone = isInAllianceZone(shooterTranslation, azLineOffset);
    Translation2d target2d =
        getTargetLocation(shooterTranslation, inAllianceZone, allianceHubLocation);
    double distanceTrimMeters = Inches.of(distanceTrimInches).in(Meters);
    Pose3d rawTargetPose3d =
        new Pose3d(target2d.getX(), target2d.getY(), targetHeight, new Rotation3d());

    // First-pass translation & physics to estimate time of flight without motion compensation
    Translation3d initialTranslation = rawTargetPose3d.minus(shooterPose3d).getTranslation();
    PhysicsResult firstPassPhysics =
        computePhysicsInternal(shooterPose3d, rawTargetPose3d, initialTranslation);
    double t = firstPassPhysics.timeToTarget;

    // Compensate target for robot motion: p_t' = p_t - v * t
    Translation2d compensatedTarget2d =
        target2d.minus(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).times(t));

    if (Math.abs(distanceTrimMeters) > 1e-9) {
      Translation2d shooterToTarget = compensatedTarget2d.minus(shooterTranslation);
      double norm = shooterToTarget.getNorm();
      if (norm > 1e-9) {
        compensatedTarget2d =
            compensatedTarget2d.plus(shooterToTarget.div(norm).times(distanceTrimMeters));
      }
    }

    Pose3d compensatedTargetPose3d =
        new Pose3d(
            compensatedTarget2d.getX(), compensatedTarget2d.getY(), targetHeight, new Rotation3d());

    // The vector from the shooter to the compensated target in field coordinates
    Translation3d translationToTarget =
        compensatedTargetPose3d.minus(shooterPose3d).getTranslation();

    // Azimuth angle to compensated target in world coordinates
    double angleRad = Math.atan2(translationToTarget.getY(), translationToTarget.getX());
    Rotation2d angleToTarget = Rotation2d.fromRadians(angleRad);
    Rotation2d turretAngleWorld = angleToTarget; // .rotateBy(robotYaw.times(-0.5));

    return new AimState(
        robotPose3d,
        shooterPose3d,
        compensatedTargetPose3d,
        translationToTarget,
        turretAngleWorld,
        speeds);
  }

  /**
   * Computes a simple ballistic model for the shot based on shooter/target poses.
   *
   * <p>Returns the initial X/Y launch velocities and time-of-flight needed for the projectile to
   * travel from the shooter to the target, assuming constant gravity and no air resistance or spin
   * effects.
   */
  private static PhysicsResult computePhysics(AimState state) {
    Pose3d shooterPose3d = state.shooterPose3d;
    Pose3d targetPose3d = state.targetPose3d;
    Translation3d translationToTarget = state.translationToTarget;
    return computePhysicsInternal(shooterPose3d, targetPose3d, translationToTarget);
  }

  private static PhysicsResult computePhysicsInternal(
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
  private static Setpoints applyOffsetsAndClamps(
      double calculatedRPM,
      PhysicsResult physics,
      Rotation2d turretAngleWorld,
      Rotation2d currentTurretAngle,
      double flywheelOffsetRPM,
      double hoodOffsetDeg,
      double turretOffsetDeg) {

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
    // double continuousTurretDeg = unwrapToNearest(desiredTurretDeg,
    // currentTurretAngle.getDegrees());
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
        Rotation2d.fromDegrees(desiredTurretDeg),
        flywheelClamped,
        hoodClamped,
        false);
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
    final ChassisSpeeds fieldRelativeSpeeds;

    AimState(
        Pose3d robotPose3d,
        Pose3d shooterPose3d,
        Pose3d targetPose3d,
        Translation3d translationToTarget,
        Rotation2d turretAngleWorld,
        ChassisSpeeds fieldRelativeSpeeds) {
      this.robotPose3d = robotPose3d;
      this.shooterPose3d = shooterPose3d;
      this.targetPose3d = targetPose3d;
      this.translationToTarget = translationToTarget;
      this.turretAngleWorld = turretAngleWorld;
      this.fieldRelativeSpeeds = fieldRelativeSpeeds;
    }
  }

  /**
   * Computes turret feedforward for tracking the motion-compensated ballistic target.
   *
   * <p>This mirrors the line-of-sight feedforward used in {@link AimShooterMathLinear}. The
   * ballistic model already computes a motion-compensated target, so this method converts the
   * relative target motion into turret angular velocity/acceleration and then into volts using the
   * identified turret gains.
   */
  private Voltage getTurretFF(AimState state, double timeOfFlight) {
    double currentTimestamp = Timer.getTimestamp();
    double timeSinceLastLoop = currentTimestamp - lastLoopTimestamp;
    double deltaTimeOfFlight = timeOfFlight - previousTOF;
    lastLoopTimestamp = currentTimestamp;
    previousTOF = timeOfFlight;

    Translation2d robotSpeedTranslation =
        new Translation2d(
            state.fieldRelativeSpeeds.vxMetersPerSecond,
            state.fieldRelativeSpeeds.vyMetersPerSecond);
    Translation2d robotAcceleration =
        robotSpeedTranslation.minus(previousRobotSpeed).div(timeSinceLastLoop);

    Translation2d virtualGoalVelocity =
        robotAcceleration
            .times(-timeOfFlight)
            .minus(robotSpeedTranslation.times(deltaTimeOfFlight));

    Translation2d shooterTranslation = state.shooterPose3d.getTranslation().toTranslation2d();
    Translation2d virtualTargetLocation = state.targetPose3d.getTranslation().toTranslation2d();
    Translation2d correctedVelocity = robotSpeedTranslation.minus(virtualGoalVelocity);
    Translation2d translationToVirtualGoal = virtualTargetLocation.minus(shooterTranslation);
    double translationNormSquared = translationToVirtualGoal.getSquaredNorm();

    double turretVelocity =
        (translationToVirtualGoal.getX() * correctedVelocity.getY()
                - translationToVirtualGoal.getY() * correctedVelocity.getX())
            / translationNormSquared;

    double angularAccelerationNumerator =
        (translationToVirtualGoal.getX() * robotAcceleration.getY()
                - translationToVirtualGoal.getY() * robotAcceleration.getX())
            / translationNormSquared;
    double radialVelocityComponent =
        (translationToVirtualGoal.getX() * correctedVelocity.getX()
                + translationToVirtualGoal.getY() * correctedVelocity.getY())
            / translationNormSquared;
    double turretAcceleration =
        angularAccelerationNumerator - (2.0 * turretVelocity * radialVelocityComponent);

    Voltage robotYawVelocityFF =
        Volts.of(
            TurretConstants.turretKv
                * (-Units.radiansToRotations(state.fieldRelativeSpeeds.omegaRadiansPerSecond)));

    previousRobotSpeed = robotSpeedTranslation;

    return robotYawVelocityFF.plus(
        Volts.of(
            TurretConstants.turretKv * turretVelocity
                + TurretConstants.turretKa * turretAcceleration));
  }

  /** Returns whether the shooter is currently considered inside the alliance zone. */
  private static boolean isInAllianceZone(Translation2d shooterTranslation, Distance azLine) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return shooterTranslation.getMeasureX().plus(azLineOffset).lte(azLine);
    } else {
      return shooterTranslation.getMeasureX().minus(azLineOffset).gte(azLine);
    }
  }

  /**
   * Returns the 2D target used by the ballistic model.
   *
   * <p>Inside the alliance zone, the hub is used and X/Y trims are applied in field space. Outside
   * the alliance zone, the closer of the predefined neutral-zone targets is selected.
   */
  private Translation2d getTargetLocation(
      Translation2d shooterTranslation, boolean inAllianceZone, Translation2d allianceHubLocation) {
    if (inAllianceZone) {
      return allianceHubLocation.plus(new Translation2d(getXTrim(), getYTrim()));
    } else {
      return getNZShootingTarget(shooterTranslation);
    }
  }

  /** Chooses the closer of the two predefined neutral-zone shooting targets. */
  private static Translation2d getNZShootingTarget(Translation2d robotTranslation) {
    boolean closerToDepot =
        robotTranslation.getDistance(QuadranglesUtil.toAllianceTranslation(nzDepotShootingTarget))
            < robotTranslation.getDistance(
                QuadranglesUtil.toAllianceTranslation(nzOutpostShootingTarget));
    if (closerToDepot) {
      return QuadranglesUtil.toAllianceTranslation(nzDepotShootingTarget);
    } else {
      return QuadranglesUtil.toAllianceTranslation(nzOutpostShootingTarget);
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

  public static class Setpoints {
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

  // ========== ShooterAimModel interface implementation ==========
  @Override
  public double getTurretAngleRot() {
    return setpoints.turretAngle.getRotations();
  }

  @Override
  public Voltage getTurretFF() {
    return turretFF;
  }

  @Override
  public Rotation2d getHoodAngle() {
    return setpoints.hoodAngle;
  }

  @Override
  public AngularVelocity getFlywheelSpeed() {
    return RPM.of(setpoints.rpm);
  }

  @Override
  public double getTurretTrimRot() {
    return Units.degreesToRotations(turretOffsetDeg);
  }

  @Override
  public void setTurretTrim(double trimRot) {
    turretOffsetDeg = Units.rotationsToDegrees(trimRot);
  }

  @Override
  public Rotation2d getHoodTrim() {
    return Rotation2d.fromDegrees(hoodOffsetDeg);
  }

  @Override
  public void setHoodTrim(Rotation2d trim) {
    hoodOffsetDeg = trim.getDegrees();
  }

  @Override
  public AngularVelocity getFlywheelTrim() {
    return RPM.of(flywheelOffsetRPM);
  }

  @Override
  public void setFlywheelTrim(AngularVelocity trim) {
    flywheelOffsetRPM = trim.in(RPM);
  }

  @Override
  public Distance getDistanceTrim() {
    return Inches.of(distanceTrimInches);
  }

  @Override
  public void setDistanceTrim(Distance trim) {
    distanceTrimInches = trim.in(Inches);
  }

  @Override
  public Distance getXTrim() {
    return Inches.of(xTrimInches);
  }

  @Override
  public void setXTrim(Distance trim) {
    xTrimInches = trim.in(Inches);
  }

  @Override
  public Distance getYTrim() {
    return Inches.of(yTrimInches);
  }

  @Override
  public void setYTrim(Distance trim) {
    yTrimInches = trim.in(Inches);
  }

  @Override
  public AngularVelocity applyFlywheelTrim(AngularVelocity baseSpeed) {
    return baseSpeed.plus(getFlywheelTrim());
  }

  @Override
  public Rotation2d applyHoodTrim(Rotation2d baseAngle) {
    return baseAngle.plus(getHoodTrim());
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
