package frc.robot.subsystems.shooter.turret;

import static frc.robot.Constants.TurretConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
  private SparkMax turretMotor;

  @AutoLogOutput private Rotation2d turretSetpoint = Rotation2d.kZero;

  public Turret() {
    turretMotor = new SparkMax(RobotMap.turretMotorCanId, MotorType.kBrushless);

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig
        .smartCurrentLimit(turretCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(turretInverted);
    turretConfig.closedLoop.pid(turretKp, turretKi, turretKd);
    turretConfig.closedLoop.feedForward.sva(turretKs, turretKv, turretKa);
    turretMotor.configure(
        turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    logMotorStats("Turret/Motor", turretMotor, false);
  }
}
