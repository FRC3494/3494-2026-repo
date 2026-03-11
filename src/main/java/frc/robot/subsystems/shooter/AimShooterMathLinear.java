package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  @Getter @AutoLogOutput private Rotation2d hoodAngle = Rotation2d.kZero;
  @Getter @AutoLogOutput private AngularVelocity flywheelSpeed = RPM.of(0);

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

  public AimShooterMathLinear(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds) {
    this.robotPose = robotPose;
    this.robotSpeeds = robotSpeeds;

    for (LinearInterpolationDataPoint dataPoint : linearInterpolationDataPoints) {
      hoodAngleMapRad.put(dataPoint.distance().in(Meters), dataPoint.hoodAngle().getRadians());
      flywheelSpeedMapRPM.put(dataPoint.distance().in(Meters), dataPoint.flywheelSpeed().in(RPM));
      timeOfFlightMap.put(dataPoint.distance().in(Meters), dataPoint.timeOfFlight().in(Seconds));
    }
  }

  @Override
  public void periodic() {
    Pose2d currentRobotPose = robotPose.get();
    ChassisSpeeds robotSpeed = robotSpeeds.get();

    Translation2d shooterTranslation = getRobotShooterTranslation(currentRobotPose);

    Translation2d allianceHubLocation = QuadranglesUtil.toAllianceTranslation(hubLocation);

    boolean inAllianceZone = isInAllianceZone(shooterTranslation, azLineOffset);
    Logger.recordOutput("AimShooterMathLinear/InAllianceZone", inAllianceZone);

    Translation2d targetLocation =
        getTargetLocation(shooterTranslation, inAllianceZone, allianceHubLocation);
    Logger.recordOutput(
        "AimShooterMathLinear/TargetLocation", new Pose2d(targetLocation, Rotation2d.kZero));

    Translation2d virtualTargetLocation =
        getVirtualGoal(shooterTranslation, robotSpeed, targetLocation);
    Logger.recordOutput(
        "AimShooterMathLinear/VirtualTargetLocation",
        new Pose2d(virtualTargetLocation, Rotation2d.kZero));

    double virtualDistanceToTarget =
        shooterTranslation.getDistance(virtualTargetLocation)
            + Units.inchesToMeters(distanceTrimInches.get());
    Logger.recordOutput("AimShooterMathLinear/VirtualDistance", Meters.of(virtualDistanceToTarget));

    turretAngleRot =
        getTurretAngleRot(virtualTargetLocation, shooterTranslation, currentRobotPose.getRotation())
            // - Units.radiansToRotations(robotSpeed.omegaRadiansPerSecond)
            + Units.degreesToRotations(turretTrimDeg.get());
    hoodAngle = getHoodAngle(inAllianceZone, virtualDistanceToTarget);
    flywheelSpeed = getFlywheelSpeed(inAllianceZone, virtualDistanceToTarget);
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
      return QuadranglesUtil.toAllianceTranslation(nzDepotShootingTarget);
    } else {
      return QuadranglesUtil.toAllianceTranslation(nzOutpostShootingTarget);
    }
  }

  private Translation2d getVirtualGoal(
      Translation2d shooterTranslation, ChassisSpeeds robotSpeed, Translation2d targetLocation) {
    double distanceToTarget =
        shooterTranslation.getDistance(targetLocation)
            + Units.inchesToMeters(distanceTrimInches.get());
    Logger.recordOutput("AimShooterMathLinear/Distance", Meters.of(distanceToTarget));

    double timeOfFlight = timeOfFlightMap.get(distanceToTarget);

    return targetLocation.minus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
            .times(timeOfFlight));
  }

  private double getTurretAngleRot(
      Translation2d targetTranslation, Translation2d shooterTranslation, Rotation2d robotYaw) {
    Translation2d translationToTarget = targetTranslation.minus(shooterTranslation);

    Rotation2d angle =
        Rotation2d.fromRadians(Math.atan2(translationToTarget.getY(), translationToTarget.getX()));
    return angle.rotateBy(robotYaw.times(-1.0)).getRotations();
  }

  private Rotation2d getHoodAngle(boolean inAllianceZone, double distanceMeters) {
    return Rotation2d.fromRadians(hoodAngleMapRad.get(distanceMeters))
        .plus(Rotation2d.fromDegrees(hoodTrimDeg.get()));
  }

  private AngularVelocity getFlywheelSpeed(boolean inAllianceZone, double distanceMeters) {
    return RPM.of(flywheelSpeedMapRPM.get(distanceMeters)).plus(RPM.of(flywheelTrimRPM.get()));
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
}
