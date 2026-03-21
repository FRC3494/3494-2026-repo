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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
  SparkFlex spinnySpinnyMotor;
  SparkFlex uppyDownyMotor;

  @Getter @AutoLogOutput AngularVelocity spinnySpinnySetpoint = RPM.of(0.0);
  @Getter @AutoLogOutput AngularVelocity uppyDownyVelocitySetpoint = RPM.of(0.0);

  @Getter @AutoLogOutput double uppyDownySetpoint = 0.0;
  @Getter @AutoLogOutput double uppyDownySetpointClamped = 0.0;

  private LoggedNetworkNumber uppyDownyP =
      new LoggedNetworkNumber("Tunable/IntakeUppyDowny/kP", uppyDownyKp);
  private LoggedNetworkNumber uppyDownyI =
      new LoggedNetworkNumber("Tunable/IntakeUppyDowny/kI", uppyDownyKi);
  private LoggedNetworkNumber uppyDownyD =
      new LoggedNetworkNumber("Tunable/IntakeUppyDowny/kD", uppyDownyKd);

  private LoggedNetworkNumber uppyDownyRaiseRPMTunable =
      new LoggedNetworkNumber("Tunable/IntakeUppyDowny/RaiseRPM", uppyDownyRaiseRPM);
  private LoggedNetworkNumber uppyDownyLowerRPMTunable =
      new LoggedNetworkNumber("Tunable/IntakeUppyDowny/LowerRPM", uppyDownyLowerRPM);

  @Getter @AutoLogOutput private Current uppyDownyFilteredCurrent = Amps.of(0);

  private final MedianFilter uppyDownyCurrentFilter =
      new MedianFilter(uppyDownCurrentSensingFilterSize);

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
    uppyDownyConfig
        .encoder
        .positionConversionFactor(uppyDownyGearRatio)
        .positionConversionFactor(uppyDownyGearRatio);
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

    boolean uppyDownyPidChanged =
        uppyDownyP.get() != uppyDownyKp
            || uppyDownyI.get() != uppyDownyKi
            || uppyDownyD.get() != uppyDownyKd;
    if (uppyDownyPidChanged) {
      setUppyDownyPID(uppyDownyP.get(), uppyDownyI.get(), uppyDownyD.get());
    }

    uppyDownyFilteredCurrent =
        Amps.of(uppyDownyCurrentFilter.calculate(uppyDownyMotor.getOutputCurrent()));
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

  public void setUppyDownyPosition(double setpoint) {
    uppyDownySetpoint = setpoint;
    uppyDownySetpointClamped = MathUtil.clamp(setpoint, uppyDownyMinPosition, uppyDownyMaxPosition);

    uppyDownyMotor
        .getClosedLoopController()
        .setSetpoint(uppyDownySetpointClamped, ControlType.kPosition);
  }

  public double getUppyDownyPosition() {
    return uppyDownyMotor.getEncoder().getPosition();
  }

  public void setUppyDownyVelocity(AngularVelocity velocity) {
    uppyDownyVelocitySetpoint = velocity;
    if (!velocity.isEquivalent(RPM.of(0))) {
      uppyDownyMotor.getClosedLoopController().setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    } else {
      uppyDownyMotor.getClosedLoopController().setSetpoint(0, ControlType.kVoltage);
    }
  }

  public void setUppyDownyOpenLoop(Voltage voltage) {
    uppyDownyMotor.setVoltage(voltage);
  }

  public void setUppyDownyCurrentLimit(Current limit) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit((int) limit.in(Amps));
    uppyDownyMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public AngularVelocity getUppyDownyRaiseRPM() {
    return RPM.of(uppyDownyRaiseRPMTunable.get());
  }

  public AngularVelocity getUppyDownyLowerRPM() {
    return RPM.of(uppyDownyLowerRPMTunable.get());
  }

  public void setUppyDownyRelativeEncoderPosition(double position) {
    uppyDownyMotor.getEncoder().setPosition(position);
  }

  private void setUppyDownyPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    uppyDownyKp = p;
    uppyDownyKi = i;
    uppyDownyKd = d;
    config.closedLoop.pid(p, i, d);
    uppyDownyMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
