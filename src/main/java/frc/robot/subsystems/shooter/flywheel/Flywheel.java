package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants.*;
import static frc.robot.util.QuadranglesUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
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

public class Flywheel extends SubsystemBase {
  private SparkFlex leftMotor;
  private SparkFlex rightMotor;

  @Getter
  @AutoLogOutput(key = "Shooter/Flywheel/FlywheelSetpoint")
  private AngularVelocity flywheelSetpoint = RPM.of(0.0);

  SysIdRoutine sysId;

  @AutoLogOutput(key = "Shooter/Flywheel/Shooting")
  private boolean shooting = false;

  public Flywheel() {
    leftMotor = new SparkFlex(RobotMap.Shooter.flywheelLeftCanId, MotorType.kBrushless);
    rightMotor = new SparkFlex(RobotMap.Shooter.flywheelRightCanId, MotorType.kBrushless);

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig
        .smartCurrentLimit(flywheelCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(flywheelInverted)
        .openLoopRampRate(flywheelRampRate.in(Seconds))
        .closedLoopRampRate(flywheelRampRate.in(Seconds))
        .secondaryCurrentLimit(115, 4);
    leftConfig
        .closedLoop
        .pid(flywheelKp, flywheelKi, flywheelKd)
        .iMaxAccum(flywheelMaxIAccum)
        .iZone(flywheelIZone);
    leftConfig.closedLoop.feedForward.sva(flywheelKs, flywheelKv, flywheelKa);
    leftConfig.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig().apply(leftConfig);
    rightConfig.follow(leftMotor, true);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setOpenLoop(voltage), null, this));

    if (tuningMode) {
      SmartDashboard.putData("Shooter/Flywheel", this);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (tuningMode) {
      // Flywheel Settings
      builder.addDoubleProperty(
          "Threshold Factor",
          () -> flywheelThresholdFactor,
          (double value) -> {
            flywheelThresholdFactor = value;
            Logger.recordOutput("Shooter/Flywheel/ThresholdFactor", value);
          });

      builder.addIntegerProperty(
          "Manual Speed",
          () -> ((long) flywheelManualSpeed.in(RPM)),
          (long value) -> {
            flywheelManualSpeed = RPM.of(value);
            Logger.recordOutput("Shooter/Flywheel/ManualSpeed", RPM.of(value));
          });

      // Flywheel PID
      builder.addDoubleArrayProperty(
          "PID",
          () -> new double[] {flywheelKp, flywheelKi, flywheelKd},
          (double[] values) -> {
            setPID(values[0], values[1], values[2]);
            Logger.recordOutput("Shooter/Flywheel/PID/kP", values[0]);
            Logger.recordOutput("Shooter/Flywheel/PID/kI", values[1]);
            Logger.recordOutput("Shooter/Flywheel/PID/kD", values[2]);
          });

      builder.addDoubleArrayProperty(
          "SVA",
          () -> new double[] {flywheelKs, flywheelKv, flywheelKa},
          (double[] values) -> {
            setSVA(values[0], values[1], values[2]);
            Logger.recordOutput("Shooter/Flywheel/PID/kS", values[0]);
            Logger.recordOutput("Shooter/Flywheel/PID/kV", values[1]);
            Logger.recordOutput("Shooter/Flywheel/PID/kA", values[2]);
          });
    }

    // Log initial values regardless of tuning mode
    Logger.recordOutput("Shooter/Flywheel/ThresholdFactor", flywheelThresholdFactor);
    Logger.recordOutput("Shooter/Flywheel/ManualSpeed", flywheelManualSpeed);
    Logger.recordOutput("Shooter/Flywheel/PID/kP", flywheelKp);
    Logger.recordOutput("Shooter/Flywheel/PID/kI", flywheelKi);
    Logger.recordOutput("Shooter/Flywheel/PID/kD", flywheelKd);
    Logger.recordOutput("Shooter/Flywheel/PID/kS", flywheelKs);
    Logger.recordOutput("Shooter/Flywheel/PID/kV", flywheelKv);
    Logger.recordOutput("Shooter/Flywheel/PID/kA", flywheelKa);

    builder.addDoubleProperty(
        "Speedup kA",
        () -> flywheelSpeedupKa,
        (double value) -> {
          flywheelSpeedupKa = value;
          Logger.recordOutput("Shooter/Flywheel/SpeedupKa", value);
        });
    Logger.recordOutput("Shooter/Flywheel/SpeedupKa", flywheelSpeedupKa);
  }

  @Override
  public void periodic() {
    if (Robot.loopCount % loggingFrequency == 0) {
      logMotorStats("Shooter/Flywheel/LeftMotor", leftMotor, false);
      logMotorStats("Shooter/Flywheel/RightMotor", rightMotor, false);
    }

    runFlywheel();
  }

  public void setVelocity(AngularVelocity velocity) {
    flywheelSetpoint = velocity;

    runFlywheel();
  }

  public void setShooting(boolean flywheelShouldRun) {
    shooting = flywheelShouldRun;

    runFlywheel();
  }

  public boolean atVelocity(double threshold) {
    return getVelocity().gte(flywheelSetpoint.times(threshold));
  }

  private void runFlywheel() {
    if (!shooting || flywheelSetpoint.isEquivalent(RPM.zero())) {
      leftMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
      return;
    }

    AngularVelocity currentVelocity = getVelocity();
    if (currentVelocity.lt(flywheelSetpoint)) {
      leftMotor
          .getClosedLoopController()
          .setSetpoint(flywheelSetpoint.in(RPM), ControlType.kVelocity);
    } else {
      leftMotor
          .getClosedLoopController()
          .setSetpoint(
              flywheelSetpoint.in(RPM),
              ControlType.kVelocity,
              ClosedLoopSlot.kSlot0,
              flywheelSpeedupKa * (flywheelSetpoint.minus(currentVelocity)).in(RPM),
              ArbFFUnits.kVoltage);
    }
  }

  public void setOpenLoop(Voltage voltage) {
    leftMotor.getClosedLoopController().setSetpoint(voltage.in(Volts), ControlType.kVoltage);
  }

  private void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    flywheelKp = p;
    flywheelKi = i;
    flywheelKd = d;
    config.closedLoop.pid(p, i, d);
    leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    flywheelKs = s;
    flywheelKv = v;
    flywheelKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public AngularVelocity getVelocity() {
    return RPM.of(leftMotor.getEncoder().getVelocity());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setOpenLoop(Volts.of(0.0))).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}
