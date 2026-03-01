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
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Hood extends SubsystemBase {
  private SparkFlex hoodMotor;

  @Getter
  @AutoLogOutput(key = "Shooter/Hood/HoodSetpoint")
  private Rotation2d hoodSetpoint = Rotation2d.kZero;

  @Setter
  @AutoLogOutput(key = "Shooter/Hood/HoodOffset")
  private Rotation2d hoodOffset = Rotation2d.kZero;

  private LoggedNetworkNumber hoodP = new LoggedNetworkNumber("Tunable/Hood/kP", hoodKp);
  private LoggedNetworkNumber hoodI = new LoggedNetworkNumber("Tunable/Hood/kI", hoodKi);
  private LoggedNetworkNumber hoodD = new LoggedNetworkNumber("Tunable/Hood/kD", hoodKd);

  private double p = hoodKp;
  private double i = hoodKi;
  private double d = hoodKd;

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

    if (hoodP.get() != p || hoodI.get() != i || hoodD.get() != d) {
      setPID(hoodP.get(), hoodI.get(), hoodD.get());
    }

    filteredCurrent = Amps.of(currentFilter.calculate(hoodMotor.getOutputCurrent()));
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
    config.closedLoop.pid(p, i, d);
    hoodMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
