package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber extends SubsystemBase {
  private SparkFlex climberMotor;

  @Getter @AutoLogOutput private Rotation2d climberSetpoint = Rotation2d.kZero;

  public Climber() {
    climberMotor = new SparkFlex(RobotMap.Climber.climberMotorCanId, MotorType.kBrushless);

    SparkFlexConfig climberConfig = new SparkFlexConfig();
    climberConfig
        .smartCurrentLimit(climberCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(climberInverted);
    climberConfig.closedLoop.pid(climberKp, climberKi, climberKd);
    climberConfig.closedLoop.feedForward.sva(climberKs, climberKv, climberKa);
    climberMotor.configure(
        climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    logMotorStats("Climber/Motor", climberMotor, false);
  }

  // Climb is positive
  public void setPosition(Rotation2d setpoint) {
    climberSetpoint = setpoint;
    climberMotor
        .getClosedLoopController()
        .setSetpoint(setpoint.getRotations(), ControlType.kVoltage);
  }
}
