package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private SparkFlex spindexerMotor;
  private SparkFlex kickerMotor;

  @AutoLogOutput private AngularVelocity spindexerSetpointRPM = RPM.of(0.0);
  @AutoLogOutput private AngularVelocity kickerSetpointRPM = RPM.of(0.0);

  SysIdRoutine spindexerSysId;
  SysIdRoutine kickerSysId;

  public Hopper() {
    spindexerMotor = new SparkFlex(RobotMap.Hopper.spindexerCanId, MotorType.kBrushless);
    kickerMotor = new SparkFlex(RobotMap.Hopper.kickerCanId, MotorType.kBrushless);

    // initialize spindexer motor config
    SparkFlexConfig spindexerConfig = new SparkFlexConfig();
    spindexerConfig
        .smartCurrentLimit(spindexerCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(spindexerInverted);
    spindexerConfig
        .encoder
        .positionConversionFactor(spindexerGearRatio)
        .velocityConversionFactor(spindexerGearRatio);
    spindexerConfig.closedLoop.pid(spindexerKp, spindexerKi, spindexerKd);
    spindexerConfig.closedLoop.feedForward.sva(spindexerKs, spindexerKv, spindexerKa);
    spindexerMotor.configure(
        spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // initialize kicker motor config
    SparkFlexConfig kickerConfig = new SparkFlexConfig();
    kickerConfig
        .smartCurrentLimit(kickerCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(kickerInverted);
    kickerConfig.closedLoop.pid(kickerKp, kickerKi, kickerKd);
    kickerConfig.closedLoop.feedForward.sva(kickerKs, kickerKv, kickerKa);
    kickerMotor.configure(
        kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    spindexerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(8),
                null,
                (state) -> Logger.recordOutput("Hopper/SpindexerSysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setSpindexerOpenLoop(voltage), null, this));
    kickerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Hopper/KickerSysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setKickerOpenLoop(voltage), null, this));
  }

  @Override
  public void periodic() {
    logMotorStats("Hopper/SpindexerMotor", spindexerMotor, false);
    logMotorStats("Hopper/KickerMotor", kickerMotor, false);
  }

  public void setSpindexerVelocity(AngularVelocity velocity) {
    spindexerSetpointRPM = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      spindexerMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      spindexerMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setKickerVelocity(AngularVelocity velocity) {
    kickerSetpointRPM = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      kickerMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      kickerMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setSpindexerOpenLoop(Voltage voltage) {
    spindexerMotor.setVoltage(voltage);
  }

  public void setKickerOpenLoop(Voltage voltage) {
    kickerMotor.setVoltage(voltage);
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
}
