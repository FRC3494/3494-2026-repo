package frc.robot.subsystems.shooter.hood;

import static frc.robot.Constants.ShooterConstants.HoodConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {
  private SparkFlex hoodMotor;

  @Getter
  @AutoLogOutput(key = "Shooter/Hood/HoodSetpoint")
  private Rotation2d hoodSetpoint = Rotation2d.kZero;

  private LoggedNetworkNumber hoodP = new LoggedNetworkNumber("Tunable/Hood/kP", hoodKp);
  private LoggedNetworkNumber hoodI = new LoggedNetworkNumber("Tunable/Hood/kI", hoodKi);
  private LoggedNetworkNumber hoodD = new LoggedNetworkNumber("Tunable/Hood/kD", hoodKd);

  private double p = hoodKp;
  private double i = hoodKi;
  private double d = hoodKd;

  public Hood() {
    hoodMotor = new SparkFlex(RobotMap.Shooter.hoodMotorCanId, MotorType.kBrushless);

    SparkFlexConfig hoodConfig = new SparkFlexConfig();
    hoodConfig.smartCurrentLimit(hoodCurrentLimit).idleMode(IdleMode.kBrake).inverted(hoodInverted);
    hoodConfig
        .encoder
        .positionConversionFactor(hoodGearRatio)
        .velocityConversionFactor(hoodGearRatio);
    hoodConfig
        .closedLoop
        .pid(hoodKp, hoodKi, hoodKd)
        .pid(hoodToZeroKp, hoodToZeroKi, hoodToZeroKd, ClosedLoopSlot.kSlot1);
    hoodConfig.closedLoop.feedForward.sva(hoodKs, hoodKv, hoodKa);
    hoodMotor.configure(
        hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    logMotorStats("Shooter/Hood/Motor", hoodMotor, false);
    logHoodInfo();
    if (hoodP.get() != p || hoodI.get() != i || hoodD.get() != d) {
      setPID(hoodP.get(), hoodI.get(), hoodD.get());
    }
  }

  public void logHoodInfo() {
    Logger.recordOutput("Hood/Setpoint", hoodSetpoint.getRotations());
  }
  public void setPosition(Rotation2d setpoint) {
    if (setpoint.getRadians() >= hoodMinAngle.getRadians()
        && setpoint.getRadians() <= hoodMaxAngle.getRadians()) {
      hoodSetpoint = setpoint;
      if (!setpoint.equals(Rotation2d.kZero)) {
        hoodMotor
            .getClosedLoopController()
            .setSetpoint(setpoint.getRotations(), ControlType.kPosition);
      } else {
        hoodMotor
            .getClosedLoopController()
            .setSetpoint(setpoint.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
      }
    }
  }

  private void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.pid(p, i, d);
    hoodMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
