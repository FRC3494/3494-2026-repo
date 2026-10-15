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
  private SparkFlex spinnySpinnyLeftMotor;
  private SparkFlex spinnySpinnyRightMotor;
  private SparkFlex uppyDownyMotor;

  @Getter
  @AutoLogOutput(key = "Intake/SpinnySpinny/Setpoint")
  private AngularVelocity spinnySpinnySetpoint = RPM.of(0.0);

  @Getter
  @AutoLogOutput(key = "Intake/UppyDowny/Setpoint")
  private double uppyDownySetpoint = 0.0;

  @Getter
  @AutoLogOutput(key = "Intake/UppyDowny/SetpointClamped")
  private double uppyDownySetpointClamped = 0.0;

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
    spinnySpinnyLeftMotor =
        new SparkFlex(RobotMap.Intake.spinnySpinnyLeftCanId, MotorType.kBrushless);
    SparkFlexConfig spinnySpinnyLeftConfig = new SparkFlexConfig();
    spinnySpinnyLeftConfig
        .smartCurrentLimit(((int) spinnySpinnyCurrentLimit.in(Amps)))
        .idleMode(IdleMode.kCoast)
        .inverted(spinnySpinnyInverted)
        .openLoopRampRate(spinnySpinnyRampRate.in(Seconds))
        .closedLoopRampRate(spinnySpinnyRampRate.in(Seconds))
        .secondaryCurrentLimit(115, 4);
    spinnySpinnyLeftConfig
        .encoder
        .positionConversionFactor(spinnySpinnyGearRatio)
        .velocityConversionFactor(spinnySpinnyGearRatio);
    spinnySpinnyLeftConfig.closedLoop.pid(spinnySpinnyKp, spinnySpinnyKi, spinnySpinnyKd);
    spinnySpinnyLeftConfig.closedLoop.feedForward.sva(
        spinnySpinnyKs, spinnySpinnyKv, spinnySpinnyKa);
    spinnySpinnyLeftMotor.configure(
        spinnySpinnyLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    spinnySpinnyRightMotor =
        new SparkFlex(RobotMap.Intake.spinnySpinnyRightCanId, MotorType.kBrushless);
    SparkFlexConfig spinnySpinnyRightConfig = new SparkFlexConfig().apply(spinnySpinnyLeftConfig);
    spinnySpinnyRightConfig.follow(spinnySpinnyLeftMotor, true);
    spinnySpinnyRightMotor.configure(
        spinnySpinnyRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    uppyDownyMotor = new SparkFlex(RobotMap.Intake.uppyDownyCanId, MotorType.kBrushless);
    SparkFlexConfig uppyDownyConfig = new SparkFlexConfig();
    uppyDownyConfig
        .smartCurrentLimit((int) uppyDownyCurrentLimit.in(Amps))
        .idleMode(IdleMode.kBrake)
        .inverted(uppyDownyInverted)
        .openLoopRampRate(uppyDownyRampRate.in(Seconds))
        .closedLoopRampRate(uppyDownyRampRate.in(Seconds))
        .secondaryCurrentLimit(115, 4);
    uppyDownyConfig
        .encoder
        .positionConversionFactor(uppyDownyGearRatio)
        .positionConversionFactor(uppyDownyGearRatio);
    uppyDownyConfig.closedLoop.pid(uppyDownyKp, uppyDownyKi, uppyDownyKd);
    uppyDownyConfig.closedLoop.feedForward.svag(uppyDownyKs, uppyDownyKv, uppyDownyKa, uppyDownyKg);
    uppyDownyMotor.configure(
        uppyDownyConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    spinnySpinnySysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SpinnySpinny/SysIdState", state.toString())),
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
          Logger.recordOutput("Intake/SpinnySpinny/Speed", RPM.of(value));
        });

    builder.addIntegerProperty(
        "SpinnySpinny/ShootingSpeed",
        () -> ((long) intakeSpinnySpinnyShootingSpeed.in(RPM)),
        (long value) -> {
          intakeSpinnySpinnyShootingSpeed = RPM.of(value);
          Logger.recordOutput("Intake/SpinnySpinny/ShootingSpeed", RPM.of(value));
        });

    builder.addDoubleProperty(
        "SpinnySpinny/ShootingReverseTime",
        () -> intakeSpinnySpinnyShootingReverseTime.in(Seconds),
        (double value) -> {
          intakeSpinnySpinnyShootingReverseTime = Seconds.of(value);
          Logger.recordOutput("Intake/SpinnySpinny/ShootingReverseTime", Seconds.of(value));
        });

    if (tuningMode) {
      // Spinny Spinny PID
      builder.addDoubleArrayProperty(
          "SpinnySpinny/PID",
          () -> new double[] {spinnySpinnyKp, spinnySpinnyKi, spinnySpinnyKd},
          (double[] values) -> {
            setSpinnySpinnyPID(values[0], values[1], values[2]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kP", values[0]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kI", values[1]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "SpinnySpinny/SVA",
          () -> new double[] {spinnySpinnyKs, spinnySpinnyKv, spinnySpinnyKa},
          (double[] values) -> {
            setSpinnySpinnySVA(values[0], values[1], values[2]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kS", values[0]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kV", values[1]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kA", values[2]);
          });
    }

    // Uppy Downy Settings
    builder.addDoubleProperty(
        "UppyDowny/DownPosition",
        () -> uppyDownyDownPosition,
        (double value) -> {
          uppyDownyDownPosition = value;
          Logger.recordOutput("Intake/UppyDowny/DownPosition", value);
        });

    builder.addDoubleProperty(
        "UppyDowny/UpPosition",
        () -> uppyDownyUpPosition,
        (double value) -> {
          uppyDownyUpPosition = value;
          Logger.recordOutput("Intake/UppyDowny/UpPosition", value);
        });

    builder.addDoubleProperty(
        "UppyDowny/Jostle Intake Up Time",
        () -> jostleIntakeUpTime,
        (double value) -> {
          jostleIntakeUpTime = value;
          Logger.recordOutput("Intake/UppyDowny/JostleIntakeUpTime", value);
        });

    builder.addDoubleProperty(
        "UppyDowny/Jostle Intake Down Time",
        () -> jostleIntakeDownTime,
        (double value) -> {
          jostleIntakeDownTime = value;
          Logger.recordOutput("Intake/UppyDowny/JostleIntakeDownTime", value);
        });

    if (tuningMode) {
      builder.addDoubleProperty(
          "UppyDowny/Tolerance",
          () -> uppyDownyTolerance,
          (double value) -> {
            uppyDownyTolerance = value;
            Logger.recordOutput("Intake/UppyDowny/Tolerance", value);
          });

      builder.addDoubleProperty(
          "UppyDowny/ManualVoltage",
          () -> uppyDownyManualVoltage.in(Volts),
          (double value) -> {
            uppyDownyManualVoltage = Volts.of(value);
            Logger.recordOutput("Intake/UppyDowny/ManualVoltage", Volts.of(value));
          });

      // Uppy Downy PID
      builder.addDoubleArrayProperty(
          "UppyDowny/PID",
          () -> new double[] {uppyDownyKp, uppyDownyKi, uppyDownyKd},
          (double[] values) -> {
            setUppyDownyPID(values[0], values[1], values[2]);
            Logger.recordOutput("Intake/UppyDowny/PID/kP", values[0]);
            Logger.recordOutput("Intake/UppyDowny/PID/kI", values[1]);
            Logger.recordOutput("Intake/UppyDowny/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "UppyDowny/SVAG",
          () -> new double[] {uppyDownyKs, uppyDownyKv, uppyDownyKa, uppyDownyKg},
          (double[] values) -> {
            setUppyDownySVAG(values[0], values[1], values[2], values[3]);
            Logger.recordOutput("Intake/UppyDowny/PID/kS", values[0]);
            Logger.recordOutput("Intake/UppyDowny/PID/kV", values[1]);
            Logger.recordOutput("Intake/UppyDowny/PID/kA", values[2]);
            Logger.recordOutput("Intake/UppyDowny/PID/kG", values[3]);
          });
    }

    // Log initial values regardless of tuning mode
    Logger.recordOutput("Intake/SpinnySpinny/Speed", intakeSpinnySpinnySpeed);
    Logger.recordOutput("Intake/SpinnySpinny/ShootingSpeed", intakeSpinnySpinnyShootingSpeed);
    Logger.recordOutput(
        "Intake/SpinnySpinny/ShootingReverseTime",
        intakeSpinnySpinnyShootingReverseTime.in(Seconds));
    Logger.recordOutput("Intake/SpinnySpinny/PID/kP", spinnySpinnyKp);
    Logger.recordOutput("Intake/SpinnySpinny/PID/kI", spinnySpinnyKi);
    Logger.recordOutput("Intake/SpinnySpinny/PID/kD", spinnySpinnyKd);
    Logger.recordOutput("Intake/SpinnySpinny/PID/kS", spinnySpinnyKs);
    Logger.recordOutput("Intake/SpinnySpinny/PID/kV", spinnySpinnyKv);
    Logger.recordOutput("Intake/SpinnySpinny/PID/kA", spinnySpinnyKa);

    Logger.recordOutput("Intake/UppyDowny/DownPosition", uppyDownyDownPosition);
    Logger.recordOutput("Intake/UppyDowny/UpPosition", uppyDownyUpPosition);
    Logger.recordOutput("Intake/UppyDowny/JostleIntakeUpTime", jostleIntakeUpTime);
    Logger.recordOutput("Intake/UppyDowny/JostleIntakeDownTime", jostleIntakeDownTime);
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
      logMotorStats("Intake/SpinnySpinny/Motor", spinnySpinnyLeftMotor, false);
      logMotorStats("Intake/UppyDowny/Motor", uppyDownyMotor, false);
    }

    spinnySpinnyFilteredCurrent =
        Amps.of(spinnySpinnyCurrentFilter.calculate(spinnySpinnyLeftMotor.getOutputCurrent()));
    uppyDownyFilteredCurrent =
        Amps.of(uppyDownyCurrentFilter.calculate(uppyDownyMotor.getOutputCurrent()));
  }

  // #region SPINNY SPINNY

  public void setSpinnySpinnyVelocity(AngularVelocity velocity) {
    spinnySpinnySetpoint = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      spinnySpinnyLeftMotor
          .getClosedLoopController()
          .setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      spinnySpinnyLeftMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setSpinnySpinnyOpenLoop(Voltage voltage) {
    spinnySpinnyLeftMotor
        .getClosedLoopController()
        .setSetpoint(voltage.in(Volts), ControlType.kVoltage);
  }

  private void setSpinnySpinnyPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    spinnySpinnyKp = p;
    spinnySpinnyKi = i;
    spinnySpinnyKd = d;
    config.closedLoop.pid(p, i, d);
    spinnySpinnyLeftMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSpinnySpinnySVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    spinnySpinnyKs = s;
    spinnySpinnyKv = v;
    spinnySpinnyKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    spinnySpinnyLeftMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // #endregion

  // #region UPPY DOWNY

  public double getUppyDownyPosition() {
    return uppyDownyMotor.getEncoder().getPosition();
  }

  @AutoLogOutput
  public boolean isUppyDownyAtSetpoint() {
    return MathUtil.isNear(uppyDownySetpointClamped, getUppyDownyPosition(), uppyDownyTolerance);
  }

  public void setUppyDownyPosition(double position) {
    uppyDownySetpoint = position;
    uppyDownySetpointClamped = MathUtil.clamp(position, uppyDownyDownPosition, uppyDownyUpPosition);

    uppyDownyMotor
        .getClosedLoopController()
        .setSetpoint(uppyDownySetpointClamped, ControlType.kPosition);
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

  private void setUppyDownySVAG(double s, double v, double a, double g) {
    SparkFlexConfig config = new SparkFlexConfig();
    uppyDownyKs = s;
    uppyDownyKv = v;
    uppyDownyKa = a;
    uppyDownyKg = g;
    config.closedLoop.feedForward.svag(s, v, a, g);
    uppyDownyMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // #endregion

}
