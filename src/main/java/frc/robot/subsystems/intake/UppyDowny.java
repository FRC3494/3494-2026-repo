package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.util.QuadranglesUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Robot;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class UppyDowny extends SubsystemBase {
  private SparkFlex motor;

  @Getter
  @AutoLogOutput(key = "Intake/UppyDowny/Setpoint")
  private double setpoint = 0.0;

  @Getter
  @AutoLogOutput(key = "Intake/UppyDowny/SetpointClamped")
  private double setpointClamped = 0.0;

  @Getter
  @AutoLogOutput(key = "Intake/UppyDowny/Motor/FilteredCurrent")
  private Current filteredCurrent = Amps.of(0);

  private final MedianFilter currentFilter = new MedianFilter(uppyDownyCurrentSensingFilterSize);

  public UppyDowny() {
    motor = new SparkFlex(RobotMap.Intake.uppyDownyCanId, MotorType.kBrushless);
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .smartCurrentLimit((int) uppyDownyCurrentLimit.in(Amps))
        .idleMode(IdleMode.kBrake)
        .inverted(uppyDownyInverted)
        .openLoopRampRate(uppyDownyRampRate.in(Seconds))
        .closedLoopRampRate(uppyDownyRampRate.in(Seconds))
        .secondaryCurrentLimit(115, 4);
    config
        .encoder
        .positionConversionFactor(uppyDownyGearRatio)
        .positionConversionFactor(uppyDownyGearRatio);
    config.closedLoop.pid(uppyDownyKp, uppyDownyKi, uppyDownyKd);
    config.closedLoop.feedForward.svag(uppyDownyKs, uppyDownyKv, uppyDownyKa, uppyDownyKg);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData("Intake/UppyDowny", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    // Uppy Downy Settings
    builder.addDoubleProperty(
        "DownPosition",
        () -> uppyDownyDownPosition,
        (double value) -> {
          uppyDownyDownPosition = value;
          Logger.recordOutput("Intake/UppyDowny/DownPosition", value);
        });

    builder.addDoubleProperty(
        "UpPosition",
        () -> uppyDownyUpPosition,
        (double value) -> {
          uppyDownyUpPosition = value;
          Logger.recordOutput("Intake/UppyDowny/UpPosition", value);
        });

    builder.addDoubleProperty(
        "Jostle Intake Up Time",
        () -> jostleIntakeUpTime.in(Seconds),
        (double value) -> {
          jostleIntakeUpTime = Seconds.of(value);
          Logger.recordOutput("Intake/UppyDowny/JostleIntakeUpTime", Seconds.of(value));
        });

    builder.addDoubleProperty(
        "Jostle Intake Down Time",
        () -> jostleIntakeDownTime.in(Seconds),
        (double value) -> {
          jostleIntakeDownTime = Seconds.of(value);
          Logger.recordOutput("Intake/UppyDowny/JostleIntakeDownTime", Seconds.of(value));
        });

    builder.addDoubleProperty(
        "Jostle Intake Voltage",
        () -> jostleIntakeVoltage.in(Volts),
        (double value) -> {
          jostleIntakeVoltage = Volts.of(value);
          Logger.recordOutput("Intake/UppyDowny/JostleIntakeVoltage", Volts.of(value));
        });

    if (tuningMode) {
      builder.addDoubleProperty(
          "Tolerance",
          () -> uppyDownyTolerance,
          (double value) -> {
            uppyDownyTolerance = value;
            Logger.recordOutput("Intake/UppyDowny/Tolerance", value);
          });

      builder.addDoubleProperty(
          "ManualVoltage",
          () -> uppyDownyManualVoltage.in(Volts),
          (double value) -> {
            uppyDownyManualVoltage = Volts.of(value);
            Logger.recordOutput("Intake/UppyDowny/ManualVoltage", Volts.of(value));
          });

      // Uppy Downy PID
      builder.addDoubleArrayProperty(
          "PID",
          () -> new double[] {uppyDownyKp, uppyDownyKi, uppyDownyKd},
          (double[] values) -> {
            setPID(values[0], values[1], values[2]);
            Logger.recordOutput("Intake/UppyDowny/PID/kP", values[0]);
            Logger.recordOutput("Intake/UppyDowny/PID/kI", values[1]);
            Logger.recordOutput("Intake/UppyDowny/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "SVAG",
          () -> new double[] {uppyDownyKs, uppyDownyKv, uppyDownyKa, uppyDownyKg},
          (double[] values) -> {
            setSVAG(values[0], values[1], values[2], values[3]);
            Logger.recordOutput("Intake/UppyDowny/PID/kS", values[0]);
            Logger.recordOutput("Intake/UppyDowny/PID/kV", values[1]);
            Logger.recordOutput("Intake/UppyDowny/PID/kA", values[2]);
            Logger.recordOutput("Intake/UppyDowny/PID/kG", values[3]);
          });
    }

    Logger.recordOutput("Intake/UppyDowny/DownPosition", uppyDownyDownPosition);
    Logger.recordOutput("Intake/UppyDowny/UpPosition", uppyDownyUpPosition);
    Logger.recordOutput("Intake/UppyDowny/JostleIntakeUpTime", jostleIntakeUpTime);
    Logger.recordOutput("Intake/UppyDowny/JostleIntakeDownTime", jostleIntakeDownTime);
    Logger.recordOutput("Intake/UppyDowny/JostleIntakeVoltage", jostleIntakeVoltage);
    Logger.recordOutput("Intake/UppyDowny/Tolerance", uppyDownyTolerance);
    Logger.recordOutput("Intake/UppyDowny/ManualVoltage", uppyDownyManualVoltage);
    Logger.recordOutput("Intake/UppyDowny/PID/kP", uppyDownyKp);
    Logger.recordOutput("Intake/UppyDowny/PID/kI", uppyDownyKi);
    Logger.recordOutput("Intake/UppyDowny/PID/kD", uppyDownyKd);
    Logger.recordOutput("Intake/UppyDowny/PID/kS", uppyDownyKs);
    Logger.recordOutput("Intake/UppyDowny/PID/kV", uppyDownyKv);
    Logger.recordOutput("Intake/UppyDowny/PID/kA", uppyDownyKa);
    Logger.recordOutput("Intake/UppyDowny/PID/kG", uppyDownyKg);
  }

  @Override
  public void periodic() {
    if (Robot.loopCount % loggingFrequency == 0) {
      logMotorStats("Intake/UppyDowny/Motor", motor, false);
    }

    filteredCurrent = Amps.of(currentFilter.calculate(motor.getOutputCurrent()));
  }

  public double getPosition() {
    return motor.getEncoder().getPosition();
  }

  @AutoLogOutput(key = "Intake/UppyDowny/IsAtSetpoint")
  public boolean isAtSetpoint() {
    return MathUtil.isNear(setpointClamped, getPosition(), uppyDownyTolerance);
  }

  public void setPosition(double position) {
    setpoint = position;
    setpointClamped = MathUtil.clamp(position, uppyDownyMinPosition, uppyDownyMaxPosition);

    motor.getClosedLoopController().setSetpoint(setpointClamped, ControlType.kPosition);
  }

  public void setOpenLoop(Voltage voltage) {
    motor.getClosedLoopController().setSetpoint(voltage.in(Volts), ControlType.kVoltage);
  }

  public void setCurrentLimit(Current limit) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit((int) limit.in(Amps));
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setRelativeEncoderPosition(double position) {
    motor.getEncoder().setPosition(position);
  }

  public void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    uppyDownyKp = p;
    uppyDownyKi = i;
    uppyDownyKd = d;
    config.closedLoop.pid(p, i, d);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSVAG(double s, double v, double a, double g) {
    SparkFlexConfig config = new SparkFlexConfig();
    uppyDownyKs = s;
    uppyDownyKv = v;
    uppyDownyKa = a;
    uppyDownyKg = g;
    config.closedLoop.feedForward.svag(s, v, a, g);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
