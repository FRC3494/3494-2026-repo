package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.HopperConstants.*;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotMap;
import frc.robot.Robot;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private SparkFlex spindexerMotor;
  private SparkFlex kickerMotor;

  @AutoLogOutput private AngularVelocity spindexerSetpoint = RPM.zero();
  @AutoLogOutput private AngularVelocity kickerSetpoint = RPM.zero();
  @AutoLogOutput private AngularVelocity kickerSetpointClamped = RPM.zero();

  @Getter
  @AutoLogOutput(key = "Hopper/SpindexerFilteredMotorCurrent")
  private Current spindexerFilteredCurrent = Amps.zero();

  private final MedianFilter spindexerCurrentFilter =
      new MedianFilter(spindexerCurrentSensingFilterSize);

  SysIdRoutine spindexerSysId;
  SysIdRoutine kickerSysId;

  public Hopper() {
    spindexerMotor = new SparkFlex(RobotMap.Hopper.spindexerCanId, MotorType.kBrushless);
    kickerMotor = new SparkFlex(RobotMap.Hopper.kickerCanId, MotorType.kBrushless);

    // initialize spindexer motor config
    SparkFlexConfig spindexerConfig = new SparkFlexConfig();
    spindexerConfig
        .smartCurrentLimit(((int) spindexerCurrentLimit.in(Amps)))
        .idleMode(IdleMode.kCoast)
        .inverted(spindexerInverted)
        .openLoopRampRate(spindexerRampRate.in(Seconds))
        .closedLoopRampRate(spindexerRampRate.in(Seconds));
    spindexerConfig
        .encoder
        .positionConversionFactor(spindexerGearRatio)
        .velocityConversionFactor(spindexerGearRatio)
        .quadratureMeasurementPeriod(25)
        .quadratureAverageDepth(16);
    spindexerConfig
        .closedLoop
        .pid(spindexerKp, spindexerKi, spindexerKd)
        .iMaxAccum(spindexerIMaxAccum)
        .iZone(spindexerIZone);
    spindexerConfig.closedLoop.feedForward.sva(spindexerKs, spindexerKv, spindexerKa);
    spindexerMotor.configure(
        spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // initialize kicker motor config
    SparkFlexConfig kickerConfig = new SparkFlexConfig();
    kickerConfig
        .smartCurrentLimit(kickerCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(kickerInverted)
        .openLoopRampRate(kickerRampRate.in(Seconds))
        .closedLoopRampRate(kickerRampRate.in(Seconds));
    kickerConfig.closedLoop.pid(kickerKp, kickerKi, kickerKd);
    kickerConfig.closedLoop.feedForward.sva(kickerKs, kickerKv, kickerKa);
    kickerMotor.configure(
        kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    spindexerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Hopper/Spindexer/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setSpindexerOpenLoop(voltage), null, this));
    kickerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Hopper/Kicker/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setKickerOpenLoop(voltage), null, this));

    if (tuningMode) {
      SmartDashboard.putData("Hopper", this);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (tuningMode) {
      // Spindexer Speeds
      builder.addIntegerProperty(
          "Spindexer/Speed (RPM)",
          () -> ((long) spindexerSpeed.in(RPM)),
          (long value) -> {
            spindexerSpeed = RPM.of(value);
            Logger.recordOutput("Hopper/Spindexer/Speed", RPM.of(value));
          });

      builder.addIntegerProperty(
          "Spindexer/Speed while Intaking",
          () -> (long) spindexerIntakingSpeed.in(RPM),
          (long value) -> {
            spindexerIntakingSpeed = RPM.of(value);
            Logger.recordOutput("Hopper/Spindexer/IntakingSpeed", RPM.of(value));
          });

      // Spindexer PID
      builder.addDoubleArrayProperty(
          "Spindexer/PID",
          () -> new double[] {spindexerKp, spindexerKi, spindexerKd},
          (double[] values) -> {
            setSpindexerPID(values[0], values[1], values[2]);
            Logger.recordOutput("Hopper/Spindexer/PID/kP", values[0]);
            Logger.recordOutput("Hopper/Spindexer/PID/kI", values[1]);
            Logger.recordOutput("Hopper/Spindexer/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "Spindexer/SVA",
          () -> new double[] {spindexerKs, spindexerKv, spindexerKa},
          (double[] values) -> {
            setSpindexerSVA(values[0], values[1], values[2]);
            Logger.recordOutput("Hopper/Spindexer/PID/kS", values[0]);
            Logger.recordOutput("Hopper/Spindexer/PID/kV", values[1]);
            Logger.recordOutput("Hopper/Spindexer/PID/kA", values[2]);
          });

      // Spindexer Current
      builder.addIntegerProperty(
          "Spindexer/Current Limit",
          () -> ((long) spindexerCurrentLimit.in(Amps)),
          (long value) -> {
            setSpindexerCurrentLimit(Amps.of(value));
            Logger.recordOutput("Hopper/Spindexer/CurrentLimit", Amps.of(value));
          });

      builder.addIntegerProperty(
          "Spindexer/Current Threshold",
          () -> ((long) spindexerCurrentLimit.minus(spindexerCurrentThreshold).in(Amps)),
          (long value) -> {
            Current threshold = spindexerCurrentLimit.minus(Amps.of(value));
            spindexerCurrentThreshold = threshold;
            Logger.recordOutput("Hopper/Spindexer/CurrentThreshold", threshold);
          });

      // Kicker Speed
      builder.addIntegerProperty(
          "Kicker/Speed",
          () -> ((long) kickerSpeed.in(RPM)),
          (long value) -> {
            kickerSpeed = RPM.of(value);
            Logger.recordOutput("Hopper/Kicker/Speed", RPM.of(value));
          });

      builder.addDoubleArrayProperty(
          "Kicker/PID",
          () -> new double[] {kickerKp, kickerKi, kickerKd},
          (double[] values) -> {
            setKickerPID(values[0], values[1], values[2]);
            Logger.recordOutput("Hopper/Kicker/PID/kP", values[0]);
            Logger.recordOutput("Hopper/Kicker/PID/kI", values[1]);
            Logger.recordOutput("Hopper/Kicker/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "Kicker/SVA",
          () -> new double[] {kickerKs, kickerKv, kickerKa},
          (double[] values) -> {
            setKickerSVA(values[0], values[1], values[2]);
            Logger.recordOutput("Hopper/Kicker/PID/kS", values[0]);
            Logger.recordOutput("Hopper/Kicker/PID/kV", values[1]);
            Logger.recordOutput("Hopper/Kicker/PID/kA", values[2]);
          });
    }

    // Log initial values regardless of tuning mode
    Logger.recordOutput("Hopper/Spindexer/Speed", spindexerSpeed);
    Logger.recordOutput("Hopper/Spindexer/IntakingSpeed", spindexerIntakingSpeed);
    Logger.recordOutput("Hopper/Spindexer/PID/kP", spindexerKp);
    Logger.recordOutput("Hopper/Spindexer/PID/kI", spindexerKi);
    Logger.recordOutput("Hopper/Spindexer/PID/kD", spindexerKd);
    Logger.recordOutput("Hopper/Spindexer/PID/kS", spindexerKs);
    Logger.recordOutput("Hopper/Spindexer/PID/kV", spindexerKv);
    Logger.recordOutput("Hopper/Spindexer/PID/kA", spindexerKa);
    Logger.recordOutput("Hopper/Spindexer/CurrentLimit", spindexerCurrentLimit);
    Logger.recordOutput("Hopper/Spindexer/CurrentThreshold", spindexerCurrentThreshold);
    Logger.recordOutput("Hopper/Kicker/Speed", kickerSpeed);
    Logger.recordOutput("Hopper/Kicker/PID/kP", kickerKp);
    Logger.recordOutput("Hopper/Kicker/PID/kI", kickerKi);
    Logger.recordOutput("Hopper/Kicker/PID/kD", kickerKd);
    Logger.recordOutput("Hopper/Kicker/PID/kS", kickerKs);
    Logger.recordOutput("Hopper/Kicker/PID/kV", kickerKv);
    Logger.recordOutput("Hopper/Kicker/PID/kA", kickerKa);
  }

  @Override
  public void periodic() {
    if (Robot.loopCount % loggingFrequency == 0) {
      logMotorStats("Hopper/Spindexer/Motor", spindexerMotor, false);
      logMotorStats("Hopper/Kicker/Motor", kickerMotor, false);
      Logger.recordOutput(
          "Hopper/SpindexerSpeedRounded", ((int) spindexerMotor.getEncoder().getVelocity()));
    }

    spindexerFilteredCurrent =
        Amps.of(spindexerCurrentFilter.calculate(spindexerMotor.getOutputCurrent()));
  }

  public void setSpindexerVelocity(AngularVelocity velocity) {
    spindexerSetpoint = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      spindexerMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      spindexerMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setKickerVelocity(AngularVelocity velocity) {
    kickerSetpoint = velocity;
    kickerSetpointClamped =
        RPM.of(MathUtil.clamp(velocity.in(RPM), -kickerMaxSpeed.in(RPM), kickerMaxSpeed.in(RPM)));

    if (!kickerSetpointClamped.isEquivalent(RPM.zero())) {
      kickerMotor
          .getClosedLoopController()
          .setSetpoint(kickerSetpointClamped.in(RPM), ControlType.kVelocity);
    } else {
      kickerMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setSpindexerOpenLoop(Voltage voltage) {
    spindexerMotor.getClosedLoopController().setSetpoint(voltage.in(Volts), ControlType.kVoltage);
  }

  public void setKickerOpenLoop(Voltage voltage) {
    kickerMotor.getClosedLoopController().setSetpoint(voltage.in(Volts), ControlType.kVoltage);
  }

  public double getSpindexerCurrent() {
    return spindexerMotor.getOutputCurrent();
  }

  public Command kickerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setKickerOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(kickerSysId.quasistatic(direction));
  }

  public Command kickerSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setKickerOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(kickerSysId.dynamic(direction));
  }

  public Command spindexerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setSpindexerOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(spindexerSysId.quasistatic(direction));
  }

  public Command spindexerSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setSpindexerOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(spindexerSysId.dynamic(direction));
  }

  private void setSpindexerPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    spindexerKp = p;
    spindexerKi = i;
    spindexerKd = d;
    config.closedLoop.pid(p, i, d);
    spindexerMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSpindexerSVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    spindexerKs = s;
    spindexerKv = v;
    spindexerKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    spindexerMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSpindexerCurrentLimit(Current limit) {
    SparkFlexConfig config = new SparkFlexConfig();
    spindexerCurrentLimit = limit;
    config.smartCurrentLimit(((int) limit.in(Amps)));
    spindexerMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setKickerPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    kickerKp = p;
    kickerKi = i;
    kickerKd = d;
    config.closedLoop.pid(p, i, d);
    kickerMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setKickerSVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    kickerKs = s;
    kickerKv = v;
    kickerKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    kickerMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
