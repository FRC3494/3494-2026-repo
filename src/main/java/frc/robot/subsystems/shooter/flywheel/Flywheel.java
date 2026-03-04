package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants.*;
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
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Flywheel extends SubsystemBase {
  private SparkFlex leftMotor;
  private SparkFlex rightMotor;

  @Getter
  @AutoLogOutput(key = "Shooter/Flywheel/FlywheelSetpoint")
  private AngularVelocity flywheelSetpoint = RPM.of(0.0);

  SysIdRoutine sysId;

  private LoggedNetworkNumber flywheelP =
      new LoggedNetworkNumber("Tunable/Flywheel/kP", flywheelKp);
  private LoggedNetworkNumber flywheelI =
      new LoggedNetworkNumber("Tunable/Flywheel/kI", flywheelKi);
  private LoggedNetworkNumber flywheelD =
      new LoggedNetworkNumber("Tunable/Flywheel/kD", flywheelKd);

  public Flywheel() {
    leftMotor = new SparkFlex(RobotMap.Shooter.flywheelLeftCanId, MotorType.kBrushless);
    rightMotor = new SparkFlex(RobotMap.Shooter.flywheelRightCanId, MotorType.kBrushless);

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig
        .smartCurrentLimit(flywheelCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(flywheelInverted);
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
  }

  @Override
  public void periodic() {
    logMotorStats("Shooter/Flywheel/LeftMotor", leftMotor, false);
    logMotorStats("Shooter/Flywheel/RightMotor", rightMotor, false);

    boolean pidChanged =
        flywheelP.get() != flywheelKp
            || flywheelI.get() != flywheelKi
            || flywheelD.get() != flywheelKd;
    if (pidChanged) {
      setPID(flywheelP.get(), flywheelI.get(), flywheelD.get());
    }
  }

  public void setVelocity(AngularVelocity velocity) {
    flywheelSetpoint = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      leftMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      leftMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setOpenLoop(Voltage voltage) {
    leftMotor.setVoltage(voltage);
  }

  private void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    flywheelKp = p;
    flywheelKi = i;
    flywheelKd = d;
    config.closedLoop.pid(p, i, d);
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
