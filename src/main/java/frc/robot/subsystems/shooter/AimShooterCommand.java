package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.QuadranglesUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class AimShooterCommand extends Command {
  private Shooter shooter;
  private Drive drive;

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
    Translation2d currentLocation = drive.getPose().getTranslation();

    Translation2d translationToTarget = shooterTarget.minus(currentLocation);
    double distanceToTarget = currentLocation.getDistance(shooterTarget); // Meters

    Rotation2d angleToTarget;
    if (translationToTarget.getX() > 0) {
      angleToTarget =
          Rotation2d.fromRadians(
              Math.atan(translationToTarget.getY() / translationToTarget.getX()));
    } else if (translationToTarget.getX() < 0) {
      angleToTarget =
          Rotation2d.fromRadians(Math.atan(translationToTarget.getY() / translationToTarget.getX()))
              .rotateBy(Rotation2d.k180deg);
    } else if (translationToTarget.getY() > 0) {
      angleToTarget = Rotation2d.kCCW_90deg;
    } else {
      angleToTarget = Rotation2d.kCW_90deg;
    }

    Rotation2d turretAngle = angleToTarget.rotateBy(drive.getRotation().times(-1.0));
    Rotation2d hoodAngle = Rotation2d.kZero;
    AngularVelocity flywheelVelocity = RPM.of(0);

    shooter.setPosition(flywheelVelocity, hoodAngle, turretAngle);
  }
}
