package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private SparkFlex hoodMotor;

  @Getter
  @AutoLogOutput(key = "Shooter/Hood/HoodSetpoint")
  private Rotation2d hoodSetpoint = Rotation2d.kZero;

  @AutoLogOutput(key = "Shooter/Hood/HoodSetpointClamped")
  private Rotation2d hoodSetpointClamped = Rotation2d.kZero;

  @AutoLogOutput(key = "Shooter/Hood/Shooting")
  private boolean shooting = false;

  @Getter
  @AutoLogOutput(key = "Shooter/Hood/FilteredMotorCurrent")
  private Current filteredCurrent = Amps.of(0);

  private final MedianFilter currentFilter = new MedianFilter(hoodCurrentSensingFilterSize);

  public Hood() {
    hoodMotor = new SparkFlex(RobotMap.Shooter.hoodMotorCanId, MotorType.kBrushless);

    SparkFlexConfig hoodConfig = new SparkFlexConfig();
    hoodConfig
        .smartCurrentLimit(hoodCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(hoodInverted)
        .openLoopRampRate(hoodRampRate.in(Seconds))
        .closedLoopRampRate(hoodRampRate.in(Seconds));
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

    Logger.recordOutput("Shooter/Hood/Motor/CurrentLimit", Amps.of(hoodCurrentLimit));

    setRelativeEncoderPosition(hoodMinAngle);

    SmartDashboard.putData("Shooter/Hood", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Manual Angle",
        hoodManualAngle::getDegrees,
        (double value) -> hoodManualAngle = Rotation2d.fromDegrees(value));
    builder.addDoubleProperty(
        "Manual Increment",
        hoodManualIncrement::getDegrees,
        (double value) -> hoodManualIncrement = Rotation2d.fromDegrees(value));

    builder.addDoubleArrayProperty(
        "PID",
        () -> new double[] {hoodKp, hoodKi, hoodKd},
        (double[] values) -> setPID(values[0], values[1], values[2]));
    builder.addDoubleArrayProperty(
        "ToZero PID",
        () -> new double[] {hoodToZeroKp, hoodToZeroKi, hoodToZeroKd},
        (double[] values) -> setToZeroPID(values[0], values[1], values[2]));
    builder.addDoubleArrayProperty(
        "SVA",
        () -> new double[] {hoodKs, hoodKv, hoodKa},
        (double[] values) -> setSVA(values[0], values[1], values[2]));
  }

  private void logSendableValues() {
    Logger.recordOutput("Shooter/Hood/PID/kP", hoodKp);
    Logger.recordOutput("Shooter/Hood/PID/kI", hoodKi);
    Logger.recordOutput("Shooter/Hood/PID/kD", hoodKd);
    Logger.recordOutput("Shooter/Hood/ToZeroPID/kP", hoodKp);
    Logger.recordOutput("Shooter/Hood/ToZeroPID/kI", hoodKi);
    Logger.recordOutput("Shooter/Hood/ToZeroPID/kD", hoodKd);
    Logger.recordOutput("Shooter/Hood/PID/kS", hoodKs);
    Logger.recordOutput("Shooter/Hood/PID/kV", hoodKv);
    Logger.recordOutput("Shooter/Hood/PID/kA", hoodKa);
  }

  @Override
  public void periodic() {
    logMotorStats("Shooter/Hood/Motor", hoodMotor, false);
    logSendableValues();

    filteredCurrent = Amps.of(currentFilter.calculate(hoodMotor.getOutputCurrent()));
  }

  public void setPosition(Rotation2d setpoint) {
    hoodSetpoint = setpoint;

    hoodSetpointClamped =
        Rotation2d.fromRotations(
            MathUtil.clamp(
                setpoint.getRotations(), hoodMinAngle.getRotations(), hoodMaxAngle.getRotations()));

    if (shooting) {
      moveToPosition(hoodSetpointClamped);
    } else {
      moveToPosition(hoodMinAngle);
    }
  }

  public void setShooting(boolean hoodShouldMove) {
    shooting = hoodShouldMove;

    if (hoodShouldMove) {
      moveToPosition(hoodSetpointClamped);
    } else {
      moveToPosition(hoodMinAngle);
    }
  }

  private void moveToPosition(Rotation2d position) {
    if (!position.equals(hoodMinAngle)) {
      hoodMotor
          .getClosedLoopController()
          .setSetpoint(position.getRotations(), ControlType.kPosition);
    } else {
      hoodMotor
          .getClosedLoopController()
          .setSetpoint(position.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }
  }

  public void setOpenLoop(Voltage voltage) {
    hoodMotor.setVoltage(voltage);
  }

  public void setCurrentLimit(Current limit) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit((int) limit.in(Amps));
    hoodMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    Logger.recordOutput("Shooter/Hood/Motor/CurrentLimit", limit);
  }

  public void setRelativeEncoderPosition(Rotation2d position) {
    hoodMotor.getEncoder().setPosition(position.getRotations());
  }

  private void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    hoodKp = p;
    hoodKi = i;
    hoodKd = d;
    config.closedLoop.pid(p, i, d);
    hoodMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setToZeroPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    hoodToZeroKp = p;
    hoodToZeroKi = i;
    hoodToZeroKd = d;
    config.closedLoop.pid(p, i, d, ClosedLoopSlot.kSlot1);
    hoodMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    hoodKs = s;
    hoodKv = v;
    hoodKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    hoodMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
