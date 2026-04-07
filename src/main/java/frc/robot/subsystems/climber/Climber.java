package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private SparkFlex climberMotor;

  @Getter @AutoLogOutput private double climberSetpoint = climberDownPosition;
  @Getter @AutoLogOutput private double climberSetpointClamped = climberDownPosition;

  @Getter @AutoLogOutput private Current filteredCurrent = Amps.of(0);

  private final MedianFilter currentFilter = new MedianFilter(climberCurrentSensingFilterSize);

  public Climber() {
    climberMotor = new SparkFlex(RobotMap.Climber.climberMotorCanId, MotorType.kBrushless);

    SparkFlexConfig climberConfig = new SparkFlexConfig();
    climberConfig
        .smartCurrentLimit(((int) climberCurrentLimit.in(Amps)))
        .idleMode(IdleMode.kBrake)
        .inverted(climberInverted)
        .openLoopRampRate(climberRampRate.in(Seconds))
        .closedLoopRampRate(climberRampRate.in(Seconds));
    climberConfig
        .encoder
        .positionConversionFactor(climberGearRatio)
        .velocityConversionFactor(climberGearRatio);
    climberConfig.closedLoop.pid(climberKp, climberKi, climberKd);
    climberConfig.closedLoop.feedForward.sva(climberKs, climberKv, climberKa);
    climberMotor.configure(
        climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    setRelativeEncoderPosition(climberDownPosition);

    SmartDashboard.putData("Climber", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO: maybe make smth with mutable doubles so I don't have to write this much boilerplate?
    builder.addDoubleProperty(
        "Up Position", () -> climberUpPosition, (double value) -> climberUpPosition = value);
    builder.addDoubleProperty(
        "Down Position", () -> climberDownPosition, (double value) -> climberDownPosition = value);
    builder.addDoubleProperty(
        "Climb Position Factor",
        () -> climbPositionFactor,
        (double value) -> climbPositionFactor = value);
    builder.addDoubleProperty(
        "Tolerance", () -> climberTolerance, (double value) -> climberTolerance = value);

    builder.addDoubleArrayProperty(
        "PID",
        () -> new double[] {climberKp, climberKi, climberKd},
        (double[] values) -> setPID(values[0], values[1], values[2]));
    builder.addDoubleArrayProperty(
        "SVA",
        () -> new double[] {climberKs, climberKv, climberKa},
        (double[] values) -> setSVA(values[0], values[1], values[2]));

    builder.addIntegerProperty(
        "Normal Current Limit",
        () -> ((long) climberCurrentLimit.in(Amps)),
        (long value) -> {
          climberCurrentLimit = Amps.of(value);
          setCurrentLimit(Amps.of(value));
        });

    builder.addDoubleProperty(
        "Ramp Rate (ms)",
        () -> climberRampRate.in(Milliseconds),
        (double value) -> setRampRate(Milliseconds.of(value)));
  }

  private void logSendableValues() {
    Logger.recordOutput("Climber/Setpoints/UpPosition", climberUpPosition);
    Logger.recordOutput("Climber/Setpoints/DownPosition", climberDownPosition);
    Logger.recordOutput("Climber/Setpoints/ClimbPositionFactor", climbPositionFactor);
    Logger.recordOutput("Climber/Setpoints/Tolerance", climberTolerance);

    Logger.recordOutput("Climber/PID/kP", climberKp);
    Logger.recordOutput("Climber/PID/kI", climberKi);
    Logger.recordOutput("Climber/PID/kD", climberKd);
    Logger.recordOutput("Climber/PID/kS", climberKs);
    Logger.recordOutput("Climber/PID/kV", climberKv);
    Logger.recordOutput("Climber/PID/kA", climberKa);

    Logger.recordOutput("Climber/NormalCurrentLimit", climberCurrentLimit);

    Logger.recordOutput("Climber/Motor/RampRate", climberRampRate);
  }

  @Override
  public void periodic() {
    logMotorStats("Climber/Motor", climberMotor, false);
    logSendableValues();

    filteredCurrent = Amps.of(currentFilter.calculate(climberMotor.getOutputCurrent()));
  }

  public double getPosition() {
    return climberMotor.getEncoder().getPosition();
  }

  // Climber down is positive
  public void setPosition(double setpoint) {
    // Min position is ~2.4 and Max position is 0, since positive is "climb" direction -> climber
    // moves down
    climberSetpoint = setpoint;
    climberSetpointClamped = MathUtil.clamp(setpoint, climberUpPosition, climberDownPosition);

    climberMotor
        .getClosedLoopController()
        .setSetpoint(climberSetpointClamped, ControlType.kPosition);
  }

  public void setOpenLoop(Voltage voltage) {
    climberMotor.setVoltage(voltage);
  }

  public void setCurrentLimit(Current limit) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit((int) limit.in(Amps));
    climberMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    Logger.recordOutput("Climber/Motor/CurrentLimit", limit);
  }

  public void setRelativeEncoderPosition(double position) {
    climberMotor.getEncoder().setPosition(position);
  }

  private void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    climberKp = p;
    climberKi = i;
    climberKd = d;
    config.closedLoop.pid(p, i, d);
    climberMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    climberKs = s;
    climberKv = v;
    climberKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    climberMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setRampRate(Time rampRate) {
    SparkFlexConfig config = new SparkFlexConfig();
    climberRampRate = rampRate;
    config.openLoopRampRate(rampRate.in(Seconds));
    config.closedLoopRampRate(rampRate.in(Seconds));
    climberMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
