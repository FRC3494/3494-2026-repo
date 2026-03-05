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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {
  private SparkFlex hoodMotor;

  @Getter
  @AutoLogOutput(key = "Shooter/Hood/HoodSetpoint")
  private Rotation2d hoodSetpoint = Rotation2d.kZero;

  @AutoLogOutput(key = "Shooter/Hood/HoodSetpointClamped")
  private Rotation2d hoodSetpointClamped = Rotation2d.kZero;

  private final LoggedNetworkNumber hoodTrimDeg =
      new LoggedNetworkNumber("Tunable/Trim/HoodTrimDeg", 0.0);

  @AutoLogOutput(key = "Shooter/Hood/Shooting")
  private boolean shooting = false;

  private LoggedNetworkNumber hoodP = new LoggedNetworkNumber("Tunable/Hood/kP", hoodKp);
  private LoggedNetworkNumber hoodI = new LoggedNetworkNumber("Tunable/Hood/kI", hoodKi);
  private LoggedNetworkNumber hoodD = new LoggedNetworkNumber("Tunable/Hood/kD", hoodKd);

  @Getter
  @AutoLogOutput(key = "Shooter/Hood/FilteredMotorCurrent")
  private Current filteredCurrent = Amps.of(0);

  private final MedianFilter currentFilter = new MedianFilter(hoodCurrentSensingFilterSize);

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

    boolean pidChanged = hoodP.get() != hoodKp || hoodI.get() != hoodKi || hoodD.get() != hoodKd;
    if (pidChanged) {
      setPID(hoodP.get(), hoodI.get(), hoodD.get());
    }

    filteredCurrent = Amps.of(currentFilter.calculate(hoodMotor.getOutputCurrent()));
  }

  public void setPosition(Rotation2d setpoint) {
    hoodSetpoint = setpoint.plus(Rotation2d.fromDegrees(hoodTrimDeg.get()));

    hoodSetpointClamped =
        Rotation2d.fromRotations(
            MathUtil.clamp(
                hoodSetpoint.getRotations(),
                hoodMinAngle.getRotations(),
                hoodMaxAngle.getRotations()));

    if (shooting) {
      moveToPosition(hoodSetpointClamped);
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
}
