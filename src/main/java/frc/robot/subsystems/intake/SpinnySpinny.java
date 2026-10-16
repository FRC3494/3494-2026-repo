package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.util.QuadranglesUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SpinnySpinny extends SubsystemBase {
  private SparkFlex leftMotor;
  private SparkFlex rightMotor;

  @Getter
  @AutoLogOutput(key = "Intake/SpinnySpinny/Setpoint")
  private AngularVelocity setpoint = RPM.of(0.0);

  @Getter
  @AutoLogOutput(key = "Intake/SpinnySpinny/State")
  private State state = State.IDLE;

  public enum State {
    IDLE,
    INTAKE,
    OUTTAKE,
    INTAKE_WHILE_SHOOTING,
    OUTTAKE_WHILE_SHOOTING
  }

  @Getter
  @AutoLogOutput(key = "Intake/SpinnySpinny/Motor/FilteredCurrent")
  private Current filteredCurrent = Amps.zero();

  private final MedianFilter currentFilter = new MedianFilter(spinnySpinnyCurrentSensingFilterSize);

  @Getter private final SysIdRoutine sysId;

  public SpinnySpinny() {
    leftMotor = new SparkFlex(RobotMap.Intake.spinnySpinnyLeftCanId, MotorType.kBrushless);
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig
        .smartCurrentLimit(((int) spinnySpinnyCurrentLimit.in(Amps)))
        .idleMode(spinnySpinnyIdleMode)
        .inverted(spinnySpinnyInverted)
        .openLoopRampRate(spinnySpinnyRampRate.in(Seconds))
        .closedLoopRampRate(spinnySpinnyRampRate.in(Seconds))
        .secondaryCurrentLimit(115, 4);
    leftConfig
        .encoder
        .positionConversionFactor(spinnySpinnyGearRatio)
        .velocityConversionFactor(spinnySpinnyGearRatio);
    leftConfig.closedLoop.pid(spinnySpinnyKp, spinnySpinnyKi, spinnySpinnyKd);
    leftConfig.closedLoop.feedForward.sva(spinnySpinnyKs, spinnySpinnyKv, spinnySpinnyKa);
    leftMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    rightMotor = new SparkFlex(RobotMap.Intake.spinnySpinnyRightCanId, MotorType.kBrushless);
    SparkFlexConfig rightConfig = new SparkFlexConfig().apply(leftConfig);
    rightConfig.follow(leftMotor, true);
    rightMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SpinnySpinny/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setOpenLoop(voltage), null, this));

    SmartDashboard.putData("Intake/SpinnySpinny", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // Spinny Spinny Speeds
    builder.addIntegerProperty(
        "Speed",
        () -> ((long) intakeSpinnySpinnySpeed.in(RPM)),
        (long value) -> {
          intakeSpinnySpinnySpeed = RPM.of(value);
          Logger.recordOutput("Intake/SpinnySpinny/Speed", RPM.of(value));
        });

    builder.addIntegerProperty(
        "ShootingSpeed",
        () -> ((long) intakeSpinnySpinnyShootingSpeed.in(RPM)),
        (long value) -> {
          intakeSpinnySpinnyShootingSpeed = RPM.of(value);
          Logger.recordOutput("Intake/SpinnySpinny/ShootingSpeed", RPM.of(value));
        });

    builder.addDoubleProperty(
        "ShootingReverseTime",
        () -> intakeSpinnySpinnyShootingReverseTime.in(Seconds),
        (double value) -> {
          intakeSpinnySpinnyShootingReverseTime = Seconds.of(value);
          Logger.recordOutput("Intake/SpinnySpinny/ShootingReverseTime", Seconds.of(value));
        });

    if (tuningMode) {
      // Spinny Spinny PID
      builder.addDoubleArrayProperty(
          "PID",
          () -> new double[] {spinnySpinnyKp, spinnySpinnyKi, spinnySpinnyKd},
          (double[] values) -> {
            setPID(values[0], values[1], values[2]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kP", values[0]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kI", values[1]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "SVA",
          () -> new double[] {spinnySpinnyKs, spinnySpinnyKv, spinnySpinnyKa},
          (double[] values) -> {
            setSVA(values[0], values[1], values[2]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kS", values[0]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kV", values[1]);
            Logger.recordOutput("Intake/SpinnySpinny/PID/kA", values[2]);
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
  }

  @Override
  public void periodic() {
    if (Robot.loopCount % loggingFrequency == 0) {
      logMotorStats("Intake/SpinnySpinny/Motor", leftMotor, false);
    }

    filteredCurrent = Amps.of(currentFilter.calculate(leftMotor.getOutputCurrent()));
  }

  public Command runState(State state) {
    return run(() -> {
          if (this.state == state) return;

          changeState(state);
        })
        .finallyDo(() -> changeState(State.IDLE))
        .withName("SpinnySpinny.Run_" + state);
  }

  public Command setState(State state) {
    return runOnce(() -> changeState(state));
  }

  private void changeState(State state) {
    this.state = state;

    switch (state) {
      case IDLE:
        stop();
        break;

      case INTAKE:
        setVelocity(intakeSpinnySpinnySpeed);
        break;

      case OUTTAKE:
        setVelocity(intakeSpinnySpinnySpeed.unaryMinus());
        break;

      case INTAKE_WHILE_SHOOTING:
        setVelocity(intakeSpinnySpinnyShootingSpeed);
        break;

      case OUTTAKE_WHILE_SHOOTING:
        setVelocity(intakeSpinnySpinnyShootingSpeed.unaryMinus());
        break;
    }
  }

  public void stop() {
    leftMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
  }

  public void setVelocity(AngularVelocity velocity) {
    setpoint = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      leftMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      leftMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setOpenLoop(Voltage voltage) {
    leftMotor.getClosedLoopController().setSetpoint(voltage.in(Volts), ControlType.kVoltage);
  }

  private void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    spinnySpinnyKp = p;
    spinnySpinnyKi = i;
    spinnySpinnyKd = d;
    config.closedLoop.pid(p, i, d);
    leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    spinnySpinnyKs = s;
    spinnySpinnyKv = v;
    spinnySpinnyKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
