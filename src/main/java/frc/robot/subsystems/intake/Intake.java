package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  SparkFlex spinnySpinnyMotor;
  SparkFlex uppyDownyMotor;

  @Getter @AutoLogOutput AngularVelocity spinnySpinnySetpoint = RPM.of(0.0);
  @Getter @AutoLogOutput Rotation2d uppyDownySetpoint = Rotation2d.kZero;

  public Intake() {
    spinnySpinnyMotor = new SparkFlex(RobotMap.Intake.spinnySpinnyCanId, MotorType.kBrushless);
    SparkFlexConfig spinnySpinnyConfig = new SparkFlexConfig();
    spinnySpinnyConfig
        .smartCurrentLimit(spinnySpinnyCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(spinnySpinnyInverted);
    spinnySpinnyConfig.closedLoop.pid(spinnySpinnyKp, spinnySpinnyKi, spinnySpinnyKd);
    spinnySpinnyConfig.closedLoop.feedForward.sva(spinnySpinnyKs, spinnySpinnyKv, spinnySpinnyKa);
    spinnySpinnyMotor.configure(
        spinnySpinnyConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    uppyDownyMotor = new SparkFlex(RobotMap.Intake.uppyDownyCanId, MotorType.kBrushless);
    SparkFlexConfig uppyDownyConfig = new SparkFlexConfig();
    uppyDownyConfig
        .smartCurrentLimit(uppyDownyCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(uppyDownyInverted);
    uppyDownyConfig.closedLoop.pid(uppyDownyKp, uppyDownyKi, uppyDownyKd);
    uppyDownyConfig.closedLoop.feedForward.sva(uppyDownyKs, uppyDownyKv, uppyDownyKa);
    uppyDownyMotor.configure(
        uppyDownyConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    logMotorStats("Intake/SpinnySpinnyMotor", spinnySpinnyMotor, false);
    logMotorStats("Intake/UppyDownyMotor", uppyDownyMotor, false);
  }

  public void setSpinnySpinnyVelocity(AngularVelocity velocity) {
    spinnySpinnySetpoint = velocity;
    spinnySpinnyMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVoltage);
  }

  public void setUppyDownyPosition(Rotation2d setpoint) {
    uppyDownySetpoint = setpoint;
    uppyDownyMotor
        .getClosedLoopController()
        .setSetpoint(setpoint.getRotations(), ControlType.kMAXMotionPositionControl);
  }
}
