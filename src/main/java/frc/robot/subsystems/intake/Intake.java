package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  SparkFlex spinnySpinnyMotor;
  SparkFlex uppyDownyMotor;

  @Getter @AutoLogOutput AngularVelocity spinnySpinnySetpoint = RPM.of(0.0);
  @Getter @AutoLogOutput Rotation2d uppyDownySetpoint = Rotation2d.kZero;

  SysIdRoutine spinnySpinnySysId;

  public Intake() {
    spinnySpinnyMotor = new SparkFlex(RobotMap.Intake.spinnySpinnyCanId, MotorType.kBrushless);
    SparkFlexConfig spinnySpinnyConfig = new SparkFlexConfig();
    spinnySpinnyConfig
        .smartCurrentLimit(spinnySpinnyCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(spinnySpinnyInverted);
    spinnySpinnyConfig
        .encoder
        .positionConversionFactor(spinnySpinnyGearRatio)
        .velocityConversionFactor(spinnySpinnyGearRatio);
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

    spinnySpinnySysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SpinnySpinnySysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setSpinnySpinnyOpenLoop(voltage), null, this));
  }

  @Override
  public void periodic() {
    logMotorStats("Intake/SpinnySpinnyMotor", spinnySpinnyMotor, false);
    logMotorStats("Intake/UppyDownyMotor", uppyDownyMotor, false);
  }

  public void setSpinnySpinnyVelocity(AngularVelocity velocity) {
    spinnySpinnySetpoint = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      spinnySpinnyMotor
          .getClosedLoopController()
          .setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      spinnySpinnyMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setUppyDownyPosition(Rotation2d setpoint) {
    uppyDownySetpoint = setpoint;
    uppyDownyMotor
        .getClosedLoopController()
        .setSetpoint(setpoint.getRotations(), ControlType.kMAXMotionPositionControl);
  }

  public void setSpinnySpinnyOpenLoop(Voltage voltage) {
    spinnySpinnyMotor.setVoltage(voltage);
  }

  public Command spinnySpinnySysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setSpinnySpinnyOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(spinnySpinnySysId.quasistatic(direction));
  }

  public Command spinnySpinnySysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setSpinnySpinnyOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(spinnySpinnySysId.dynamic(direction));
  }
}
