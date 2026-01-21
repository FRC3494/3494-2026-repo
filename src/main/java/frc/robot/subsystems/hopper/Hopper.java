package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hopper extends SubsystemBase {
  private SparkMax spindexerMotor;
  private SparkMax feederMotor;

  @AutoLogOutput private AngularVelocity spindexerSetpointRPM = RPM.of(0.0);
  @AutoLogOutput private AngularVelocity feederSetpointRPM = RPM.of(0.0);

  public Hopper() {
    spindexerMotor = new SparkMax(RobotMap.hopperSpindexerCanId, MotorType.kBrushless);
    feederMotor = new SparkMax(RobotMap.hopperFeederCanId, MotorType.kBrushless);

    // initialize spindexer motor config
    SparkMaxConfig spindexerConfig = new SparkMaxConfig();
    spindexerConfig
        .smartCurrentLimit(hopperSpindexerCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(hopperSpindexerInverted);
    // spindexerConfig.closedLoop.feedForward.sva(spindexerKs, spindexerKv, spindexerKa);
    spindexerConfig.closedLoop.pid(spindexerKp, spindexerKi, spindexerKd);
    spindexerMotor.configure(
        spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // initialize feeder motor config
    SparkMaxConfig feederConfig = new SparkMaxConfig();
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
    logMotorStats("Hopper/SpindexerMotor", spindexerMotor, false);
    logMotorStats("Hopper/FeederMotor", feederMotor, false);
  }

  public void setSpindexerVelocity(AngularVelocity velocity) {
    spindexerSetpointRPM = velocity;
    spindexerMotor
        .getClosedLoopController()
        .setSetpoint(velocity.in(RPM), ControlType.kMAXMotionVelocityControl);
  }

  public void setFeederVelocity(AngularVelocity velocity) {
    feederSetpointRPM = velocity;
    feederMotor
        .getClosedLoopController()
        .setSetpoint(velocity.in(RPM), ControlType.kMAXMotionVelocityControl);
  }
}
