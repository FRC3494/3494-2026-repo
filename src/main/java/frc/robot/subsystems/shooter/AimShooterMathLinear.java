package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

  @Getter @AutoLogOutput private double turretAngleRot = 0.0;
  @Getter @AutoLogOutput private Rotation2d hoodAngle = Rotation2d.kZero;
  @Getter @AutoLogOutput private AngularVelocity flywheelSpeed = RPM.of(0);

  @AutoLogOutput
  private Translation2d targetLocation = QuadranglesUtil.toAllianceTranslation(hubLocation);

  private final InterpolatingDoubleTreeMap hoodAngleMapRad = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flywheelSpeedMapRPM = new InterpolatingDoubleTreeMap();

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

  public AimShooterMathLinear(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;

    for (LinearInterpolationDataPoint dataPoint : linearInterpolationDataPoints) {
      hoodAngleMapRad.put(dataPoint.distance().in(Meters), dataPoint.hoodAngle().getRadians());
      flywheelSpeedMapRPM.put(dataPoint.distance().in(Meters), dataPoint.flywheelSpeed().in(RPM));
    }
  }

  @Override
  public void periodic() {
    Pose2d currentRobotPose = robotPose.get();
    Translation2d shooterTranslation = getRobotShooterTranslation(currentRobotPose);

    Translation2d allianceHubLocation = QuadranglesUtil.toAllianceTranslation(hubLocation);

    boolean inAllianceZone;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      inAllianceZone = shooterTranslation.getMeasureX().lte(allianceHubLocation.getMeasureX());
    } else {
      inAllianceZone = shooterTranslation.getMeasureX().gte(allianceHubLocation.getMeasureX());
    }
    Logger.recordOutput("AimShooterMathLinear/InAllianceZone", inAllianceZone);

    if (inAllianceZone) {
      targetLocation = allianceHubLocation.plus(new Translation2d(getXTrim(), getYTrim()));
    } else {
      targetLocation = getNZShootingTarget(shooterTranslation);
    }

    turretAngleRot =
        getTurretAngleRot(shooterTranslation, currentRobotPose.getRotation())
            + Units.degreesToRotations(turretTrimDeg.get());

    double distanceToTarget =
        shooterTranslation.getDistance(targetLocation)
            + Units.inchesToMeters(distanceTrimInches.get());
    Logger.recordOutput("AimShooterMathLinear/Distance", Meters.of(distanceToTarget));

    hoodAngle =
        Rotation2d.fromRadians(hoodAngleMapRad.get(distanceToTarget))
            .plus(Rotation2d.fromDegrees(hoodTrimDeg.get()));
    flywheelSpeed =
        RPM.of(flywheelSpeedMapRPM.get(distanceToTarget)).plus(RPM.of(flywheelTrimRPM.get()));
  }

  private Translation2d getRobotShooterTranslation(Pose2d currentRobotPose) {
    Transform2d shooterTransform =
        new Transform2d(
            new Translation2d(ShooterConstants.shooterX, ShooterConstants.shooterY),
            new Rotation2d());

    return currentRobotPose.transformBy(shooterTransform).getTranslation();
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

  private double getTurretAngleRot(Translation2d shooterTranslation, Rotation2d robotYaw) {
    Translation2d translationToTarget = targetLocation.minus(shooterTranslation);

    Rotation2d angle =
        Rotation2d.fromRadians(Math.atan2(translationToTarget.getY(), translationToTarget.getX()));
    return angle.rotateBy(robotYaw.times(-1.0)).getRotations();
  }

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
