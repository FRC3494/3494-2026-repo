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
import frc.robot.util.QuadranglesUtil;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class AimShooterMathLinear extends SubsystemBase {
  private final Supplier<Pose2d> robotPose;

  @Getter @AutoLogOutput private double turretAngleRot = 0.0;
  @Getter @AutoLogOutput private Rotation2d hoodAngle = Rotation2d.kZero;
  @Getter @AutoLogOutput private AngularVelocity flywheelSpeed = RPM.of(0);

  @AutoLogOutput
  private Translation2d targetLocation = QuadranglesUtil.toAllianceTranslation(hubLocation);

  @AutoLogOutput private Distance azNZBoundary = targetLocation.getMeasureX();

  private final InterpolatingDoubleTreeMap hoodAngleMapRad = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flywheelSpeedMapRPM = new InterpolatingDoubleTreeMap();

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

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (shooterTranslation.getMeasureX().lte(azNZBoundary)) {
        targetLocation = QuadranglesUtil.toAllianceTranslation(hubLocation);
      } else {
        targetLocation = getNZShootingTarget(shooterTranslation);
      }
    } else {
      if (shooterTranslation.getMeasureX().gte(azNZBoundary)) {
        targetLocation = QuadranglesUtil.toAllianceTranslation(hubLocation);
      } else {
        targetLocation = getNZShootingTarget(shooterTranslation);
      }
    }

    turretAngleRot = getTurretAngle(shooterTranslation);

    double distanceToTarget = shooterTranslation.getDistance(targetLocation);
    hoodAngle = Rotation2d.fromRadians(hoodAngleMapRad.get(distanceToTarget));
    flywheelSpeed = RPM.of(flywheelSpeedMapRPM.get(distanceToTarget));
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

  private double getTurretAngle(Translation2d shooterTranslation) {
    Translation2d translationToTarget = targetLocation.minus(shooterTranslation);

    double angleRad = Math.atan2(translationToTarget.getY(), translationToTarget.getX());
    return Units.radiansToRotations(angleRad);
  }
}
