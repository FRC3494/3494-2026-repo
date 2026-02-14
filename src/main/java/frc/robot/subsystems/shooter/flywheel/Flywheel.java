package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.util.LogUtil.logMotorStats;

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
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Flywheel extends SubsystemBase {
  private SparkMax leftMotor;
  private SparkMax rightMotor;

  @Getter @AutoLogOutput private AngularVelocity flywheelSetpoint = RPM.of(0.0);

  public Flywheel() {
    leftMotor = new SparkMax(RobotMap.shooterLeftCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(RobotMap.shooterRightCanId, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .smartCurrentLimit(shooterCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(shooterInverted);
    leftConfig.closedLoop.pid(shooterKp, shooterKi, shooterKd);
    leftConfig.closedLoop.feedForward.sva(shooterKs, shooterKv, shooterKa);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig().apply(leftConfig);
    rightConfig.inverted(!shooterInverted).follow(leftMotor);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    logMotorStats("Flywheel/LeftMotor", leftMotor, false);
    logMotorStats("Flywheel/RightMotor", rightMotor, false);
  }

  public void setVelocity(AngularVelocity velocity) {
    flywheelSetpoint = velocity;
    leftMotor
        .getClosedLoopController()
        .setSetpoint(velocity.in(RPM), ControlType.kMAXMotionVelocityControl);
  }
}
