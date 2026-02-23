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
  private SparkFlex feederMotor;

  @AutoLogOutput private AngularVelocity spindexerSetpointRPM = RPM.of(0.0);
  @AutoLogOutput private AngularVelocity feederSetpointRPM = RPM.of(0.0);

  SysIdRoutine feederSysId;

  public Hopper() {
    spindexerMotor = new SparkFlex(RobotMap.Hopper.spindexerCanId, MotorType.kBrushless);
    feederMotor = new SparkFlex(RobotMap.Hopper.feederCanId, MotorType.kBrushless);

    // initialize spindexer motor config
    SparkFlexConfig spindexerConfig = new SparkFlexConfig();
    spindexerConfig
        .smartCurrentLimit(spindexerCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(spindexerInverted);
    spindexerConfig.closedLoop.pid(spindexerKp, spindexerKi, spindexerKd);
    spindexerConfig.closedLoop.feedForward.sva(spindexerKs, spindexerKv, spindexerKa);
    spindexerMotor.configure(
        spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // initialize feeder motor config
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig
        .smartCurrentLimit(feederCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(feederInverted);
    feederConfig.closedLoop.pid(feederKp, feederKi, feederKd);
    feederConfig.closedLoop.feedForward.sva(feederKs, feederKv, feederKa);
    feederMotor.configure(
        feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    feederSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("Shooter/Hopper/FeederSysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setFeederOpenLoop(voltage), null, this));
  }

  @Override
  public void periodic() {
    logMotorStats("Shooter/Hopper/SpindexerMotor", spindexerMotor, false);
    logMotorStats("Shooter/Hopper/FeederMotor", feederMotor, false);
  }

  public void setSpindexerVelocity(AngularVelocity velocity) {
    spindexerSetpointRPM = velocity;
    spindexerMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVoltage);
  }

  public void setFeederVelocity(AngularVelocity velocity) {
    feederSetpointRPM = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      feederMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      feederMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setFeederOpenLoop(Voltage voltage) {
    feederMotor.setVoltage(voltage);
  }

  public Command feederSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setFeederOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(feederSysId.quasistatic(direction));
  }

  public Command feederSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setFeederOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(feederSysId.dynamic(direction));
  }
}
