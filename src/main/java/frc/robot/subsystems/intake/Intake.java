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
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotMap;
import frc.robot.Robot;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  SparkFlex spinnySpinnyMotor;
  SparkFlex uppyDownyMotor;

  @Getter @AutoLogOutput AngularVelocity spinnySpinnySetpoint = RPM.of(0.0);
  @Getter @AutoLogOutput AngularVelocity uppyDownyVelocitySetpoint = RPM.of(0.0);

  @Getter @AutoLogOutput double uppyDownySetpoint = 0.0;
  @Getter @AutoLogOutput double uppyDownySetpointClamped = 0.0;

  @Getter
  @AutoLogOutput(key = "Intake/SpinnySpinny/Motor/FilteredCurrent")
  private Current spinnySpinnyFilteredCurrent = Amps.zero();

  @Getter
  @AutoLogOutput(key = "Intake/UppyDowny/Motor/FilteredCurrent")
  private Current uppyDownyFilteredCurrent = Amps.of(0);

  private final MedianFilter spinnySpinnyCurrentFilter =
      new MedianFilter(spinnySpinnyCurrentSensingFilterSize);
  private final MedianFilter uppyDownyCurrentFilter =
      new MedianFilter(uppyDownyCurrentSensingFilterSize);

  @Getter private final SysIdRoutine spinnySpinnySysId;

  public Intake() {
    spinnySpinnyMotor = new SparkFlex(RobotMap.Intake.spinnySpinnyCanId, MotorType.kBrushless);
    SparkFlexConfig spinnySpinnyConfig = new SparkFlexConfig();
    spinnySpinnyConfig
        .smartCurrentLimit(((int) spinnySpinnyCurrentLimit.in(Amps)))
        .idleMode(IdleMode.kCoast)
        .inverted(spinnySpinnyInverted)
        .openLoopRampRate(spinnySpinnyRampRate.in(Seconds))
        .closedLoopRampRate(spinnySpinnyRampRate.in(Seconds))
        .secondaryCurrentLimit(115, 4);
    spinnySpinnyConfig
        .encoder
        .positionConversionFactor(spinnySpinnyGearRatio)
        .velocityConversionFactor(spinnySpinnyGearRatio);
    spinnySpinnyConfig.closedLoop.pid(spinnySpinnyKp, spinnySpinnyKi, spinnySpinnyKd);
    spinnySpinnyConfig.closedLoop.feedForward.sva(spinnySpinnyKs, spinnySpinnyKv, spinnySpinnyKa);
    spinnySpinnyMotor.configure(
        spinnySpinnyConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    uppyDownyMotor = new SparkFlex(RobotMap.Intake.uppyDownyCanId, MotorType.kBrushless);
    SparkFlexConfig uppyDownyConfig = new SparkFlexConfig();
    uppyDownyConfig
        .smartCurrentLimit(uppyDownyCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(uppyDownyInverted)
        .openLoopRampRate(uppyDownyRampRate.in(Seconds))
        .closedLoopRampRate(uppyDownyRampRate.in(Seconds))
        .secondaryCurrentLimit(115, 4);
    uppyDownyConfig
        .encoder
        .positionConversionFactor(uppyDownyGearRatio)
        .positionConversionFactor(uppyDownyGearRatio);
    uppyDownyConfig.closedLoop.pid(uppyDownyKp, uppyDownyKi, uppyDownyKd);
    uppyDownyConfig.closedLoop.feedForward.sva(uppyDownyKs, uppyDownyKv, uppyDownyKa);
    uppyDownyMotor.configure(
        uppyDownyConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    spinnySpinnySysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SpinnySpinnySysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setSpinnySpinnyOpenLoop(voltage), null, this));

    SmartDashboard.putData("Intake", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // Spinny Spinny Speeds
    builder.addIntegerProperty(
        "SpinnySpinny/Speed",
        () -> ((long) intakeSpinnySpinnySpeed.in(RPM)),
        (long value) -> {
          intakeSpinnySpinnySpeed = RPM.of(value);
          Logger.recordOutput("SpinnySpinny/Speed", RPM.of(value));
        });

    builder.addIntegerProperty(
        "SpinnySpinny/ShootingSpeed",
        () -> ((long) intakeSpinnySpinnyShootingSpeed.in(RPM)),
        (long value) -> {
          intakeSpinnySpinnyShootingSpeed = RPM.of(value);
          Logger.recordOutput("SpinnySpinny/ShootingSpeed", RPM.of(value));
        });

    if (tuningMode) {
      // Spinny Spinny PID
      builder.addDoubleArrayProperty(
          "SpinnySpinny/PID",
          () -> new double[] {spinnySpinnyKp, spinnySpinnyKi, spinnySpinnyKd},
          (double[] values) -> {
            setSpinnySpinnyPID(values[0], values[1], values[2]);
            Logger.recordOutput("SpinnySpinny/PID/kP", values[0]);
            Logger.recordOutput("SpinnySpinny/PID/kI", values[1]);
            Logger.recordOutput("SpinnySpinny/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "SpinnySpinny/SVA",
          () -> new double[] {spinnySpinnyKs, spinnySpinnyKv, spinnySpinnyKa},
          (double[] values) -> {
            setSpinnySpinnySVA(values[0], values[1], values[2]);
            Logger.recordOutput("SpinnySpinny/PID/kS", values[0]);
            Logger.recordOutput("SpinnySpinny/PID/kV", values[1]);
            Logger.recordOutput("SpinnySpinny/PID/kA", values[2]);
          });
    }

    // Uppy Downy Settings
    builder.addDoubleProperty(
        "UppyDowny/Raise RPM",
        () -> uppyDownyRaiseRPM,
        (double value) -> {
          uppyDownyRaiseRPM = value;
          Logger.recordOutput("UppyDowny/RaiseRPM", value);
        });

    builder.addDoubleProperty(
        "UppyDowny/Lower RPM",
        () -> uppyDownyLowerRPM,
        (double value) -> {
          uppyDownyLowerRPM = value;
          Logger.recordOutput("UppyDowny/LowerRPM", value);
        });

    builder.addDoubleProperty(
        "UppyDowny/Jostle Intake Up Time",
        () -> jostleIntakeUpTime,
        (double value) -> {
          jostleIntakeUpTime = value;
          Logger.recordOutput("UppyDowny/JostleIntakeUpTime", value);
        });

    builder.addDoubleProperty(
        "UppyDowny/Jostle Intake Down Time",
        () -> jostleIntakeDownTime,
        (double value) -> {
          jostleIntakeDownTime = value;
          Logger.recordOutput("UppyDowny/JostleIntakeDownTime", value);
        });

    if (tuningMode) {
      // Uppy Downy PID
      builder.addDoubleArrayProperty(
          "UppyDowny/PID",
          () -> new double[] {uppyDownyKp, uppyDownyKi, uppyDownyKd},
          (double[] values) -> {
            setUppyDownyPID(values[0], values[1], values[2]);
            Logger.recordOutput("UppyDowny/PID/kP", values[0]);
            Logger.recordOutput("UppyDowny/PID/kI", values[1]);
            Logger.recordOutput("UppyDowny/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "UppyDowny/SVA",
          () -> new double[] {uppyDownyKs, uppyDownyKv, uppyDownyKa},
          (double[] values) -> {
            setUppyDownySVA(values[0], values[1], values[2]);
            Logger.recordOutput("UppyDowny/PID/kS", values[0]);
            Logger.recordOutput("UppyDowny/PID/kV", values[1]);
            Logger.recordOutput("UppyDowny/PID/kA", values[2]);
          });
    }

    // Log initial values regardless of tuning mode
    Logger.recordOutput("SpinnySpinny/Speed", intakeSpinnySpinnySpeed);
    Logger.recordOutput("SpinnySpinny/ShootingSpeed", intakeSpinnySpinnyShootingSpeed);
    Logger.recordOutput("SpinnySpinny/PID/kP", spinnySpinnyKp);
    Logger.recordOutput("SpinnySpinny/PID/kI", spinnySpinnyKi);
    Logger.recordOutput("SpinnySpinny/PID/kD", spinnySpinnyKd);
    Logger.recordOutput("SpinnySpinny/PID/kS", spinnySpinnyKs);
    Logger.recordOutput("SpinnySpinny/PID/kV", spinnySpinnyKv);
    Logger.recordOutput("SpinnySpinny/PID/kA", spinnySpinnyKa);
    Logger.recordOutput("UppyDowny/RaiseRPM", uppyDownyRaiseRPM);
    Logger.recordOutput("UppyDowny/LowerRPM", uppyDownyLowerRPM);
    Logger.recordOutput("UppyDowny/JostleIntakeUpTime", jostleIntakeUpTime);
    Logger.recordOutput("UppyDowny/JostleIntakeDownTime", jostleIntakeDownTime);
    Logger.recordOutput("UppyDowny/PID/kP", uppyDownyKp);
    Logger.recordOutput("UppyDowny/PID/kI", uppyDownyKi);
    Logger.recordOutput("UppyDowny/PID/kD", uppyDownyKd);
    Logger.recordOutput("UppyDowny/PID/kS", uppyDownyKs);
    Logger.recordOutput("UppyDowny/PID/kV", uppyDownyKv);
    Logger.recordOutput("UppyDowny/PID/kA", uppyDownyKa);
  }

  @Override
  public void periodic() {
    if (Robot.loopCount % loggingFrequency == 0) {
      logMotorStats("Intake/SpinnySpinnyMotor", spinnySpinnyMotor, false);
      logMotorStats("Intake/UppyDownyMotor", uppyDownyMotor, false);
    }

    spinnySpinnyFilteredCurrent =
        Amps.of(spinnySpinnyCurrentFilter.calculate(spinnySpinnyMotor.getOutputCurrent()));
    uppyDownyFilteredCurrent =
        Amps.of(uppyDownyCurrentFilter.calculate(uppyDownyMotor.getOutputCurrent()));
  }

  public void setSpinnySpinnyVelocity(AngularVelocity velocity) {
    spinnySpinnySetpoint = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      spinnySpinnyMotor
          .getClosedLoopController()
          .setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      spinnySpinnyMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setSpinnySpinnyOpenLoop(Voltage voltage) {
    spinnySpinnyMotor
        .getClosedLoopController()
        .setSetpoint(voltage.in(Volts), ControlType.kVoltage);
  }

  public double getUppyDownyPosition() {
    return uppyDownyMotor.getEncoder().getPosition();
  }

  public void setUppyDownyVelocity(AngularVelocity velocity) {
    uppyDownyVelocitySetpoint = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      uppyDownyMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      uppyDownyMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setUppyDownyOpenLoop(Voltage voltage) {
    uppyDownyMotor.getClosedLoopController().setSetpoint(voltage.in(Volts), ControlType.kVoltage);
  }

  public void setUppyDownyCurrentLimit(Current limit) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit((int) limit.in(Amps));
    uppyDownyMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setUppyDownyRelativeEncoderPosition(double position) {
    uppyDownyMotor.getEncoder().setPosition(position);
  }

  private void setUppyDownyPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    uppyDownyKp = p;
    uppyDownyKi = i;
    uppyDownyKd = d;
    config.closedLoop.pid(p, i, d);
    uppyDownyMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setUppyDownySVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    uppyDownyKs = s;
    uppyDownyKv = v;
    uppyDownyKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    uppyDownyMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSpinnySpinnyPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    spinnySpinnyKp = p;
    spinnySpinnyKi = i;
    spinnySpinnyKd = d;
    config.closedLoop.pid(p, i, d);
    spinnySpinnyMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSpinnySpinnySVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    spinnySpinnyKs = s;
    spinnySpinnyKv = v;
    spinnySpinnyKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    spinnySpinnyMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
