package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ShooterConstants.TurretConstants.*;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.LinearInterpolationDataPoint;
import frc.robot.util.QuadranglesUtil;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Interpolation-based aiming model for the shooter.
 *
 * <p>This implementation uses empirically tuned lookup/interpolation tables rather than solving a
 * ballistic model directly. On each loop it:
 *
 * <ol>
 *   <li>Computes the current shooter position from robot pose.
 *   <li>Selects a field target using zone logic:
 *       <ul>
 *         <li>alliance-zone shots target the alliance hub with optional X/Y trim
 *         <li>neutral-zone shots target the closer of the depot/outpost targets
 *       </ul>
 *   <li>Applies distance trim to the lookup distance.
 *   <li>Uses interpolated maps to get hood angle, flywheel speed, and time of flight.
 *   <li>Motion-compensates the target using the interpolated time of flight.
 *   <li>Applies turret/hood/flywheel trims to the final setpoints and computes turret feedforward.
 * </ol>
 */
public class AimShooterMathLinear extends SubsystemBase implements ShooterAimModel {
  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotSpeeds;

  @Getter @AutoLogOutput private double turretAngleRot = 0.0;
  @Getter @AutoLogOutput private Voltage turretFF = Volts.of(0.0);
  @Getter @AutoLogOutput private Rotation2d hoodAngle = Rotation2d.kZero;
  @Getter @AutoLogOutput private AngularVelocity flywheelSpeed = RPM.of(0);

  /**
   * Container for all shooter setpoints, mirroring {@link AimShooterMath.Setpoints}.
   *
   * <p>This keeps the public API of {@link AimShooterMathLinear} unchanged while allowing the
   * internal logic to work with a single, immutable object that groups related values.
   */
  private record LinearSetpoints(
      double turretAngleRot,
      Voltage turretFF,
      Rotation2d hoodAngle,
      AngularVelocity flywheelSpeed) {}

  private final MedianFilter turretSetpointFilter = new MedianFilter(turretSetpointFilterSize);

  private final InterpolatingDoubleTreeMap hoodAngleMapRad = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flywheelSpeedMapRPM = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  private final LoggedNetworkNumber turretTrimDeg =
      new LoggedNetworkNumber("Tunable/Trim/TurretTrimDeg");
  private final LoggedNetworkNumber hoodTrimDeg =
      new LoggedNetworkNumber("Tunable/Trim/HoodTrimDeg");
  private final LoggedNetworkNumber flywheelTrimRPM =
      new LoggedNetworkNumber("Tunable/Trim/FlywheelTrimRPM");
  private final LoggedNetworkNumber distanceTrimInches =
      new LoggedNetworkNumber("Tunable/Trim/DistanceTrimInches");
  private final LoggedNetworkNumber xTrimInches =
      new LoggedNetworkNumber("Tunable/Trim/XTrimInches");
  private final LoggedNetworkNumber yTrimInches =
      new LoggedNetworkNumber("Tunable/Trim/YTrimInches");

  private double lastLoopTimestamp;
  private double previousTOF = 0.0;
  private Translation2d previousRobotSpeed = new Translation2d();

  private String currentTarget;

  public AimShooterMathLinear(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds) {
    this.robotPose = robotPose;
    this.robotSpeeds = robotSpeeds;

    for (LinearInterpolationDataPoint dataPoint : linearInterpolationDataPoints) {
      hoodAngleMapRad.put(dataPoint.distance().in(Meters), dataPoint.hoodAngle().getRadians());
      flywheelSpeedMapRPM.put(dataPoint.distance().in(Meters), dataPoint.flywheelSpeed().in(RPM));
      timeOfFlightMap.put(dataPoint.distance().in(Meters), dataPoint.timeOfFlight().in(Seconds));
    }

    lastLoopTimestamp = Timer.getTimestamp();
  }

  @Override
  public void periodic() {
    Pose2d currentRobotPose = robotPose.get();
    ChassisSpeeds robotSpeed = robotSpeeds.get();

    // 1) Build the current aim state (positions, target locations, distances, TOF)
    AimState state = buildAimState(currentRobotPose, robotSpeed);

    // 2) From that state, compute the shooter setpoints
    LinearSetpoints setpoints = computeSetpoints(state);

    // 3) Publish setpoints to fields used elsewhere and log key telemetry
    turretAngleRot = setpoints.turretAngleRot();
    turretFF = setpoints.turretFF();
    hoodAngle = setpoints.hoodAngle();
    flywheelSpeed = setpoints.flywheelSpeed();

    logAimShooterLinearStats(state, setpoints);
  }

  /**
   * Immutable snapshot of the geometry and timing used for linear interpolation aiming.
   *
   * <p>This is analogous to {@link AimShooterMath.AimState} but tailored to the interpolation-based
   * shooter model.
   */
  private record AimState(
      Pose2d currentRobotPose,
      ChassisSpeeds robotSpeed,
      Translation2d shooterTranslation,
      Translation2d targetLocation,
      Translation2d virtualTargetLocation,
      double distanceToTarget,
      double virtualDistanceToTarget,
      double timeOfFlight,
      boolean inAllianceZone) {}

  /**
   * Builds the target and timing state used by the interpolation-based model.
   *
   * <p>This collects the selected target, the raw/virtual distances, and the interpolated
   * time-of-flight used for motion compensation.
   */
  private AimState buildAimState(Pose2d currentRobotPose, ChassisSpeeds robotSpeed) {
    Translation2d shooterTranslation = getRobotShooterTranslation(currentRobotPose);
    Logger.recordOutput("AimShooterMathLinear/shooterTranslation", shooterTranslation);

    Translation2d allianceHubLocation = QuadranglesUtil.toAllianceTranslation(hubLocation);

    boolean inAllianceZone = isInAllianceZone(shooterTranslation, azLine);
    Logger.recordOutput("AimShooterMathLinear/InAllianceZone", inAllianceZone);

    Translation2d targetLocation =
        getTargetLocation(shooterTranslation, inAllianceZone, allianceHubLocation);
    Logger.recordOutput(
        "AimShooterMathLinear/TargetLocation", new Pose2d(targetLocation, Rotation2d.kZero));

    double distanceToTarget =
        shooterTranslation.getDistance(targetLocation)
            + Units.inchesToMeters(distanceTrimInches.get());
    Logger.recordOutput("AimShooterMathLinear/Distance", Meters.of(distanceToTarget));
    double timeOfFlight = timeOfFlightMap.get(distanceToTarget);

    Translation2d virtualTargetLocation = getVirtualGoal(timeOfFlight, robotSpeed, targetLocation);
    Logger.recordOutput(
        "AimShooterMathLinear/VirtualTargetLocation",
        new Pose2d(virtualTargetLocation, Rotation2d.kZero));

    double virtualDistanceToTarget =
        shooterTranslation.getDistance(virtualTargetLocation)
            + Units.inchesToMeters(distanceTrimInches.get());
    Logger.recordOutput("AimShooterMathLinear/VirtualDistance", Meters.of(virtualDistanceToTarget));

    return new AimState(
        currentRobotPose,
        robotSpeed,
        shooterTranslation,
        targetLocation,
        virtualTargetLocation,
        distanceToTarget,
        virtualDistanceToTarget,
        timeOfFlight,
        inAllianceZone);
  }

  /**
   * Computes final shooter outputs from the current aim state.
   *
   * <p>Hood angle, flywheel speed, and time of flight come from interpolation maps; turret angle
   * comes from target geometry and includes filtering plus feedforward.
   */
  private LinearSetpoints computeSetpoints(AimState state) {
    double filteredTurretAngleRot =
        turretSetpointFilter.calculate(
            getTurretAngleRot(
                    state.virtualTargetLocation(),
                    state.shooterTranslation(),
                    state.currentRobotPose().getRotation())
                + Units.degreesToRotations(turretTrimDeg.get()));

    Voltage computedTurretFF =
        getTurretFF(
            state.shooterTranslation(),
            state.virtualTargetLocation(),
            state.timeOfFlight(),
            state.robotSpeed());

    Rotation2d computedHoodAngle =
        getHoodAngle(state.inAllianceZone(), state.virtualDistanceToTarget())
            .plus(Rotation2d.fromDegrees(hoodTrimDeg.get()));

    AngularVelocity computedFlywheelSpeed =
        getFlywheelSpeed(state.inAllianceZone(), state.virtualDistanceToTarget())
            .plus(RPM.of(flywheelTrimRPM.get()));

    debugLogging();

    return new LinearSetpoints(
        filteredTurretAngleRot, computedTurretFF, computedHoodAngle, computedFlywheelSpeed);
  }

  /**
   * Emits detailed telemetry about the current interpolation-based aim computation.
   *
   * <p>This mirrors {@link AimShooterMath#logAimShooterStats} but for the linear model.
   */
  private static void logAimShooterLinearStats(AimState state, LinearSetpoints set) {
    double horizontalDistanceMeters =
        state.shooterTranslation().getDistance(state.targetLocation());
    double virtualHorizontalDistanceMeters =
        state.shooterTranslation().getDistance(state.virtualTargetLocation());

    Logger.recordOutput("AimShooterLinear/Distance/HorizontalMeters", horizontalDistanceMeters);
    Logger.recordOutput(
        "AimShooterLinear/Distance/VirtualHorizontalMeters", virtualHorizontalDistanceMeters);
    Logger.recordOutput("AimShooterLinear/Distance/TimeOfFlightSec", state.timeOfFlight());

    Logger.recordOutput("AimShooterLinear/Angles/TurretRot", set.turretAngleRot());
    Logger.recordOutput("AimShooterLinear/Angles/HoodSetpointDeg", set.hoodAngle().getDegrees());

    Logger.recordOutput("AimShooterLinear/Setpoints/FlywheelRPM", set.flywheelSpeed().in(RPM));
    Logger.recordOutput("AimShooterLinear/Setpoints/TurretFFVolts", set.turretFF().in(Volts));
  }

  // ==================== CALCULATIONS ====================
  /** Returns the shooter's field-relative translation based on the robot pose. */
  private Translation2d getRobotShooterTranslation(Pose2d currentRobotPose) {
    Transform2d shooterTransform =
        new Transform2d(
            new Translation2d(ShooterConstants.shooterX, ShooterConstants.shooterY),
            new Rotation2d());

    return currentRobotPose.transformBy(shooterTransform).getTranslation();
  }

  /** Returns whether the shooter is inside the alliance zone. */
  private boolean isInAllianceZone(Translation2d shooterTranslation, Distance azLineBlue) {
    // ! Flips the robot location AGAIN (back to alliance-relative coordinates essentially)
    return QuadranglesUtil.toAllianceTranslation(shooterTranslation).getMeasureX().lte(azLineBlue);
  }

  /**
   * Selects the current 2D target.
   *
   * <p>Alliance-zone shots target the hub and apply X/Y trim in field space. Neutral-zone shots use
   * the closer of the depot/outpost aiming targets.
   */
  private Translation2d getTargetLocation(
      Translation2d shooterTranslation, boolean inAllianceZone, Translation2d allianceHubLocation) {
    if (inAllianceZone) {
      return allianceHubLocation.plus(new Translation2d(getXTrim(), getYTrim()));
    } else {
      return getNZShootingTarget(shooterTranslation);
    }
  }

  /** Chooses the closer of the predefined neutral-zone shooting targets. */
  private Translation2d getNZShootingTarget(Translation2d robotTranslation) {
    boolean closerToDepot =
        robotTranslation.getDistance(QuadranglesUtil.toAllianceTranslation(nzDepotShootingTarget))
            < robotTranslation.getDistance(
                QuadranglesUtil.toAllianceTranslation(nzOutpostShootingTarget));
    if (closerToDepot) {
      currentTarget = "Depot";
      return QuadranglesUtil.toAllianceTranslation(nzDepotShootingTarget);
    } else {
      currentTarget = "Outpost";
      return QuadranglesUtil.toAllianceTranslation(nzOutpostShootingTarget);
    }
  }

  /** Returns the motion-compensated target using the interpolated time of flight. */
  private Translation2d getVirtualGoal(
      double timeOfFlight, ChassisSpeeds robotSpeed, Translation2d targetLocation) {
    return targetLocation.minus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
            .times(timeOfFlight));
  }

  /** Returns the turret setpoint in rotations relative to robot yaw. */
  private double getTurretAngleRot(
      Translation2d targetTranslation, Translation2d shooterTranslation, Rotation2d robotYaw) {
    Translation2d translationToTarget = targetTranslation.minus(shooterTranslation);

    double angle =
        Units.radiansToRotations(
            Math.atan2(translationToTarget.getY(), translationToTarget.getX()));
    return angle - robotYaw.getRotations();
  }

  /**
   * Computes turret feedforward for tracking the motion-compensated target.
   *
   * <p>The position loop decides <em>where</em> the turret should point. This feedforward estimates
   * how much additional voltage the turret will need to track that moving line-of-sight smoothly.
   * The calculation is based on three ideas:
   *
   * <ol>
   *   <li>Estimate how robot velocity and acceleration move the virtual goal over time.
   *   <li>Convert the target-relative translational motion into turret angular velocity and angular
   *       acceleration.
   *   <li>Convert those angular demands into voltage using the identified {@code turretKv} and
   *       {@code turretKa} gains, plus a compensation term for chassis yaw rate.
   * </ol>
   *
   * <p>Note: this implementation was adapted from prior FTC code and is kept behaviorally the same
   * here, but the intermediate values are spelled out to make the geometry easier to follow.
   */
  private Voltage getTurretFF(
      Translation2d shooterTranslation,
      Translation2d virtualTargetLocation,
      double timeOfFlight,
      ChassisSpeeds robotSpeed) {
    // Measure loop timing and TOF change so we can estimate first derivatives.
    double currentTimestamp = Timer.getTimestamp();
    double timeSinceLastLoop = currentTimestamp - lastLoopTimestamp;
    double deltaTimeOfFlight = timeOfFlight - previousTOF;
    lastLoopTimestamp = currentTimestamp;
    previousTOF = timeOfFlight;

    // Robot velocity/acceleration in the field plane.
    Translation2d robotSpeedTranslation =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d robotAcceleration =
        robotSpeedTranslation.minus(previousRobotSpeed).div(timeSinceLastLoop);

    // Estimate how the motion-compensated ("virtual") target itself is moving.
    // The virtual target shifts because robot velocity changes and because time of flight changes.
    Translation2d virtualGoalVelocity =
        robotAcceleration
            .times(-timeOfFlight)
            .minus(robotSpeedTranslation.times(deltaTimeOfFlight));

    // Relative translational motion between shooter and virtual goal.
    Translation2d correctedVelocity = robotSpeedTranslation.minus(virtualGoalVelocity);
    Translation2d translationToVirtualGoal = virtualTargetLocation.minus(shooterTranslation);
    double translationNormSquared = translationToVirtualGoal.getSquaredNorm();

    // Angular rate of the line-of-sight vector to the virtual goal.
    // For r = (x, y) and v = (vx, vy), d(theta)/dt = (x*vy - y*vx) / |r|^2.
    double turretVelocity =
        (translationToVirtualGoal.getX() * correctedVelocity.getY()
                - translationToVirtualGoal.getY() * correctedVelocity.getX())
            / translationNormSquared;

    // Angular acceleration of that same line-of-sight vector.
    // This is the time derivative of the angular-rate expression above.
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

    // Chassis yaw rotates the whole robot underneath the turret, so compensate that directly.
    Voltage robotYawVelocityFF =
        Volts.of(turretKv * (-Units.radiansToRotations(robotSpeed.omegaRadiansPerSecond)));

    previousRobotSpeed = robotSpeedTranslation;

    // Standard velocity + acceleration feedforward model in turret-angle units.
    return robotYawVelocityFF.plus(
        Volts.of(turretKv * turretVelocity + turretKa * turretAcceleration));
  }

  /** Returns the interpolated hood angle for the given distance. */
  private Rotation2d getHoodAngle(boolean inAllianceZone, double distanceMeters) {
    return Rotation2d.fromRadians(hoodAngleMapRad.get(distanceMeters));
  }

  /** Returns the interpolated flywheel speed for the given distance. */
  private AngularVelocity getFlywheelSpeed(boolean inAllianceZone, double distanceMeters) {
    return RPM.of(flywheelSpeedMapRPM.get(distanceMeters));
  }

  // ==================== TRIM ====================
  /** Returns the turret trim in rotations. */
  public double getTurretTrimRot() {
    return Units.degreesToRotations(turretTrimDeg.get());
  }

  public void setTurretTrim(double trimRot) {
    turretTrimDeg.set(Units.rotationsToDegrees(trimRot));
  }

  public Rotation2d getHoodTrim() {
    return Rotation2d.fromDegrees(hoodTrimDeg.get());
  }

  public void setHoodTrim(Rotation2d trim) {
    hoodTrimDeg.set(trim.getDegrees());
  }

  public AngularVelocity getFlywheelTrim() {
    return RPM.of(flywheelTrimRPM.get());
  }

  public void setFlywheelTrim(AngularVelocity trim) {
    flywheelTrimRPM.set(trim.in(RPM));
  }

  public Distance getDistanceTrim() {
    return Inches.of(distanceTrimInches.get());
  }

  public void setDistanceTrim(Distance trim) {
    distanceTrimInches.set(trim.in(Inches));
  }

  public Distance getXTrim() {
    return Inches.of(xTrimInches.get());
  }

  public void setXTrim(Distance trim) {
    xTrimInches.set(trim.in(Inches));
  }

  public Distance getYTrim() {
    return Inches.of(yTrimInches.get());
  }

  public void setYTrim(Distance trim) {
    yTrimInches.set(trim.in(Inches));
  }

  public void debugLogging() {
    Logger.recordOutput("AimShooterLinear/Debug/Target", currentTarget);
  }

  @Override
  public AngularVelocity applyFlywheelTrim(AngularVelocity baseSpeed) {
    return baseSpeed.plus(getFlywheelTrim());
  }

  @Override
  public Rotation2d applyHoodTrim(Rotation2d baseAngle) {
    return baseAngle.plus(getHoodTrim());
  }
}
