package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Climber extends SubsystemBase {
  private SparkFlex climberMotor;

  @Getter @AutoLogOutput private double climberSetpoint = 0;

  private LoggedNetworkNumber climberP = new LoggedNetworkNumber("Tunable/Climber/kP", climberKp);
  private LoggedNetworkNumber climberI = new LoggedNetworkNumber("Tunable/Climber/kI", climberKi);
  private LoggedNetworkNumber climberD = new LoggedNetworkNumber("Tunable/Climber/kD", climberKd);

  @Getter @AutoLogOutput private Current filteredCurrent = Amps.of(0);

  private final MedianFilter currentFilter = new MedianFilter(climberCurrentSensingFilterSize);

  public Climber() {
    climberMotor = new SparkFlex(RobotMap.Climber.climberMotorCanId, MotorType.kBrushless);

    SparkFlexConfig climberConfig = new SparkFlexConfig();
    climberConfig
        .smartCurrentLimit(climberCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(climberInverted);
    climberConfig
        .encoder
        .positionConversionFactor(climberGearRatio)
        .velocityConversionFactor(climberGearRatio);
    climberConfig.closedLoop.pid(climberKp, climberKi, climberKd);
    climberConfig.closedLoop.feedForward.sva(climberKs, climberKv, climberKa);
    climberMotor.configure(
        climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    logMotorStats("Climber/Motor", climberMotor, false);

    boolean pidChanged =
        climberP.get() != climberKp || climberI.get() != climberKi || climberD.get() != climberKd;
    if (pidChanged) {
      setPID(climberP.get(), climberI.get(), climberD.get());
    }

    filteredCurrent = Amps.of(currentFilter.calculate(climberMotor.getOutputCurrent()));
  }

  public double getPosition() {
    return climberMotor.getEncoder().getPosition();
  }

  // Climber down is positive
  public void setPosition(double setpoint) {
    // Min position is ~2.4 and Max position is 0, since positive is "climb" direction -> climber
    // moves down
    if (setpoint <= climberMinPosition && setpoint >= climberMaxPosition) {
      climberSetpoint = setpoint;
      climberMotor.getClosedLoopController().setSetpoint(setpoint, ControlType.kPosition);
    }
  }

  public void setOpenLoop(Voltage voltage) {
    climberMotor.setVoltage(voltage);
  }

  public void setCurrentLimit(Current limit) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit((int) limit.in(Amps));
    climberMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setRelativeEncoderPosition(double position) {
    climberMotor.getEncoder().setPosition(position);
  }

  private void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    climberKp = p;
    climberKi = i;
    climberKd = d;
    config.closedLoop.pid(p, i, d);
    climberMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
