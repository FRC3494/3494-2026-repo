package frc.robot.subsystems.shooter.hopper;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShooterConstants.HopperConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hopper extends SubsystemBase {
  private SparkFlex spindexerMotor;
  private SparkFlex feederMotor;

  @AutoLogOutput private AngularVelocity spindexerSetpointRPM = RPM.of(0.0);
  @AutoLogOutput private AngularVelocity feederSetpointRPM = RPM.of(0.0);

  public Hopper() {
    spindexerMotor = new SparkFlex(RobotMap.Shooter.hopperSpindexerCanId, MotorType.kBrushless);
    feederMotor = new SparkFlex(RobotMap.Shooter.hopperFeederCanId, MotorType.kBrushless);

    // initialize spindexer motor config
    SparkFlexConfig spindexerConfig = new SparkFlexConfig();
    spindexerConfig
        .smartCurrentLimit(hopperSpindexerCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(hopperSpindexerInverted);
    // spindexerConfig.closedLoop.feedForward.sva(spindexerKs, spindexerKv, spindexerKa);
    spindexerConfig.closedLoop.pid(spindexerKp, spindexerKi, spindexerKd);
    spindexerMotor.configure(
        spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // initialize feeder motor config
    SparkFlexConfig feederConfig = new SparkFlexConfig();
    feederConfig
        .smartCurrentLimit(hopperFeederCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(hopperFeederInverted);
    // feederConfig.closedLoop.feedForward.sva(feederKs, feederKv, feederKa);
    feederConfig.closedLoop.pid(feederKp, feederKi, feederKd);
    feederMotor.configure(
        feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    feederMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVoltage);
  }
}
