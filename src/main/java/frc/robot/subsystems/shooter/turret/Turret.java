package frc.robot.subsystems.shooter.turret;

import static frc.robot.Constants.ShooterConstants.TurretConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
  private SparkMax turretMotor;

  @Getter @AutoLogOutput private Rotation2d turretSetpoint = Rotation2d.kZero;

  public Turret() {
    turretMotor = new SparkMax(RobotMap.Shooter.turretMotorCanId, MotorType.kBrushless);

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig
        .smartCurrentLimit(turretCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(turretInverted);
    turretConfig.closedLoop.pid(turretKp, turretKi, turretKd);
    turretConfig.closedLoop.feedForward.sva(turretKs, turretKv, turretKa);
    turretConfig
        .encoder
        .positionConversionFactor(turretGearRatio)
        .velocityConversionFactor(turretGearRatio);
    turretMotor.configure(
        turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    logMotorStats("Shooter/Turret/Motor", turretMotor, false);
  }

  public void setPosition(Rotation2d setpoint) {
    turretSetpoint = setpoint;
    turretMotor
        .getClosedLoopController()
        .setSetpoint(setpoint.getRotations(), ControlType.kMAXMotionPositionControl);
  }

  public void setTurretSetpoint(Rotation2d setpoint) {
    turretSetpoint = setpoint;
    turretMotor
        .getClosedLoopController()
        .setSetpoint(setpoint.getRotations(), ControlType.kMAXMotionPositionControl);
  }
}
