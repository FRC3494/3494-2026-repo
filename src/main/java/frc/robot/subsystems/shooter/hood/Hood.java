package frc.robot.subsystems.shooter.hood;

import static frc.robot.Constants.ShooterConstants.HoodConstants.*;
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

public class Hood extends SubsystemBase {
  private SparkMax hoodMotor;

  @Getter @AutoLogOutput private Rotation2d hoodSetpoint = Rotation2d.kZero;

  public Hood() {
    hoodMotor = new SparkMax(RobotMap.Shooter.hoodMotorCanId, MotorType.kBrushless);

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig.smartCurrentLimit(hoodCurrentLimit).idleMode(IdleMode.kBrake).inverted(hoodInverted);
    hoodConfig.closedLoop.pid(hoodKp, hoodKi, hoodKd);
    hoodConfig.closedLoop.feedForward.sva(hoodKs, hoodKv, hoodKa);
    hoodMotor.configure(
        hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    logMotorStats("Shooter/Hood/Motor", hoodMotor, false);
  }

  public void setPosition(Rotation2d setpoint) {
    hoodSetpoint = setpoint;
    hoodMotor
        .getClosedLoopController()
        .setSetpoint(setpoint.getRotations(), ControlType.kMAXMotionPositionControl);
  }
}
