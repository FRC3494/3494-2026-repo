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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

public class AimShooterMathLinear extends SubsystemBase {
  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> robotSpeeds;

  @Getter @AutoLogOutput private double turretAngleRot = 0.0;
  @Getter @AutoLogOutput private Voltage turretFF = Volts.of(0.0);
  @Getter @AutoLogOutput private Rotation2d hoodAngle = Rotation2d.kZero;
  @Getter @AutoLogOutput private AngularVelocity flywheelSpeed = RPM.of(0);

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

    Translation2d shooterTranslation = getRobotShooterTranslation(currentRobotPose);

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

    turretAngleRot =
        turretSetpointFilter.calculate(
            getTurretAngleRot(
                    virtualTargetLocation, shooterTranslation, currentRobotPose.getRotation())
                + Units.degreesToRotations(turretTrimDeg.get()));
    turretFF = getTurretFF(shooterTranslation, virtualTargetLocation, timeOfFlight, robotSpeed);

    hoodAngle =
        getHoodAngle(inAllianceZone, virtualDistanceToTarget)
            .plus(Rotation2d.fromDegrees(hoodTrimDeg.get()));
    flywheelSpeed =
        getFlywheelSpeed(inAllianceZone, virtualDistanceToTarget)
            .plus(RPM.of(flywheelTrimRPM.get()));

    debugLogging();
  }

  // ==================== CALCULATIONS ====================
  private Translation2d getRobotShooterTranslation(Pose2d currentRobotPose) {
    Transform2d shooterTransform =
        new Transform2d(
            new Translation2d(ShooterConstants.shooterX, ShooterConstants.shooterY),
            new Rotation2d());

    return currentRobotPose.transformBy(shooterTransform).getTranslation();
  }

  // azLine is the X coordinate of the line between NZ and AZ
  private boolean isInAllianceZone(Translation2d shooterTranslation, Distance azLine) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return shooterTranslation.getMeasureX().plus(azLineOffset).lte(azLine);
    } else {
      return shooterTranslation.getMeasureX().minus(azLineOffset).gte(azLine);
    }
  }

  private Translation2d getTargetLocation(
      Translation2d shooterTranslation, boolean inAllianceZone, Translation2d allianceHubLocation) {
    if (inAllianceZone) {
      return allianceHubLocation.plus(new Translation2d(getXTrim(), getYTrim()));
    } else {
      return getNZShootingTarget(shooterTranslation);
    }
  }

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

  private Translation2d getVirtualGoal(
      double timeOfFlight, ChassisSpeeds robotSpeed, Translation2d targetLocation) {
    return targetLocation.minus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
            .times(timeOfFlight));
  }

  private double getTurretAngleRot(
      Translation2d targetTranslation, Translation2d shooterTranslation, Rotation2d robotYaw) {
    Translation2d translationToTarget = targetTranslation.minus(shooterTranslation);

    double angle =
        Units.radiansToRotations(
            Math.atan2(translationToTarget.getY(), translationToTarget.getX()));
    return angle - robotYaw.getRotations();
  }

  private Voltage getTurretFF(
      Translation2d shooterTranslation,
      Translation2d virtualTargetLocation,
      double timeOfFlight,
      ChassisSpeeds robotSpeed) {
    // ! Copied from FTC code, I don't understand how this works at all lol
    double timeSinceLastLoop = Timer.getTimestamp() - lastLoopTimestamp;
    double rateOfChangeOfTOF = timeOfFlight - previousTOF;
    lastLoopTimestamp = Timer.getTimestamp();
    previousTOF = timeOfFlight;

    Translation2d robotSpeedTranslation =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d robotAcceleration =
        robotSpeedTranslation.minus(previousRobotSpeed).div(timeSinceLastLoop);
    Translation2d virtualGoalVelocity =
        robotAcceleration
            .times(-timeOfFlight)
            .minus(robotSpeedTranslation.times(rateOfChangeOfTOF));
    Translation2d correctedVelocity = robotSpeedTranslation.minus(virtualGoalVelocity);
    Translation2d translationToVirtualGoal = virtualTargetLocation.minus(shooterTranslation);

    double turretVelocity =
        (translationToVirtualGoal.getX() * correctedVelocity.getY()
                - translationToVirtualGoal.getY() * correctedVelocity.getX())
            / translationToVirtualGoal.getSquaredNorm();
    double turretAcceleration =
        ((translationToVirtualGoal.getX() * robotAcceleration.getY()
                    - translationToVirtualGoal.getY() * robotAcceleration.getX())
                / translationToVirtualGoal.getSquaredNorm())
            - (2.0
                * turretVelocity
                * ((translationToVirtualGoal.getX() * correctedVelocity.getX()
                        + translationToVirtualGoal.getY() * correctedVelocity.getY())
                    / translationToVirtualGoal.getSquaredNorm()));

    // TODO: might need to be different units for yaw velocity
    // Voltage robotYawVelocityFF =
    //     Volts.of(
    //         turretKv
    //             *
    // (-Units.radiansPerSecondToRotationsPerMinute(robotSpeed.omegaRadiansPerSecond)));
    Voltage robotYawVelocityFF =
        Volts.of(turretKv * (-Units.radiansToRotations(robotSpeed.omegaRadiansPerSecond)));

    previousRobotSpeed = robotSpeedTranslation;

    return robotYawVelocityFF.plus(
        Volts.of(turretKv * turretVelocity + turretKa * turretAcceleration));
  }

  private Rotation2d getHoodAngle(boolean inAllianceZone, double distanceMeters) {
    return Rotation2d.fromRadians(hoodAngleMapRad.get(distanceMeters));
  }

  private AngularVelocity getFlywheelSpeed(boolean inAllianceZone, double distanceMeters) {
    return RPM.of(flywheelSpeedMapRPM.get(distanceMeters));
  }

  // ==================== TRIM ====================
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
}
