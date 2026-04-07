package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.TurretConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private SparkFlex turretMotor;

  @Getter
  @AutoLogOutput(key = "Shooter/Turret/TurretSetpoint", unit = "rotations")
  private double turretSetpointRot = 0.0;

  @Getter
  @AutoLogOutput(key = "Shooter/Turret/TurretSetpointClamped", unit = "rotations")
  private double turretSetpointClampedRot = 0.0;

  @Setter
  @AutoLogOutput(key = "Shooter/Turret/ArbFF")
  private Voltage turretArbFF = Volts.of(0.0);

  SysIdRoutine turretSysId;

  public Turret() {
    turretMotor = new SparkFlex(RobotMap.Shooter.turretMotorCanId, MotorType.kBrushless);

    SparkFlexConfig turretConfig = new SparkFlexConfig();
    turretConfig
        .smartCurrentLimit(turretCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(turretInverted)
        .openLoopRampRate(turretRampRate.in(Seconds))
        .closedLoopRampRate(turretRampRate.in(Seconds));
    turretConfig
        .closedLoop
        .pid(turretKp, turretKi, turretKd)
        .iMaxAccum(turretIMaxAccumRot)
        .iZone(turretIZoneRot)
        .allowedClosedLoopError(turretPositionToleranceRot, ClosedLoopSlot.kSlot0);
    turretConfig.closedLoop.feedForward.sva(turretKs, turretKv, turretKa);
    turretConfig
        .encoder
        .positionConversionFactor(turretGearRatio)
        .velocityConversionFactor(turretGearRatio);
    turretMotor.configure(
        turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(0.1),
                Volts.of(1),
                null,
                (state) -> Logger.recordOutput("Shooter/Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setOpenLoop(voltage), null, this));

    if (Math.abs(turretMotor.getEncoder().getPosition()) <= 1E-5) {
      setRelativeEncoderPosition(turretRezeroLocationRot);
    }

    SmartDashboard.putData("Shooter/Turret", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Rezero Location",
        () -> Units.rotationsToDegrees(turretRezeroLocationRot),
        (double value) -> turretRezeroLocationRot = Units.degreesToRotations(value));
    builder.addDoubleProperty(
        "Shooting Tolerance",
        () -> Units.rotationsToDegrees(turretShootingToleranceRot),
        (double value) -> turretShootingToleranceRot = Units.degreesToRotations(value));

    builder.addDoubleArrayProperty(
        "PID",
        () -> new double[] {turretKp, turretKi, turretKd},
        (double[] values) -> setPID(values[0], values[1], values[2]));
    builder.addDoubleProperty(
        "IMaxAccum (deg)",
        () -> Units.rotationsToDegrees(turretIMaxAccumRot),
        (double value) -> setPIDIntegralConstants(Units.degreesToRotations(value), turretIZoneRot));
    builder.addDoubleProperty(
        "IZone (deg)",
        () -> Units.rotationsToDegrees(turretIZoneRot),
        (double value) ->
            setPIDIntegralConstants(turretIMaxAccumRot, Units.degreesToRotations(value)));
    builder.addDoubleArrayProperty(
        "SVA",
        () -> new double[] {turretKs, turretKv, turretKa},
        (double[] values) -> setSVA(values[0], values[1], values[2]));

    builder.addDoubleProperty(
        "Cable Retractor Start Location",
        () -> Units.rotationsToDegrees(turretCableRetractorStartRot),
        (double value) -> turretCableRetractorStartRot = Units.degreesToRotations(value));
    builder.addDoubleProperty(
        "Cable Retractor CW FF",
        () -> turretCableRetractorFFCW.in(Volts),
        (double value) -> turretCableRetractorFFCW = Volts.of(value));
    builder.addDoubleProperty(
        "Cable Retractor CCW FF",
        () -> turretCableRetractorFFCCW.in(Volts),
        (double value) -> turretCableRetractorFFCCW = Volts.of(value));

    builder.addDoubleProperty(
        "Manual Increment",
        () -> Units.rotationsToDegrees(turretManualIncrementRot),
        (double value) -> turretManualIncrementRot = Units.degreesToRotations(value));
  }

  private void logSendableValues() {
    Logger.recordOutput("Shooter/Turret/RezeroLocation", turretRezeroLocationRot);
    Logger.recordOutput("Shooter/Turret/ShootingTolerance", turretShootingToleranceRot);

    Logger.recordOutput("Shooter/Turret/PID/kP", turretKp);
    Logger.recordOutput("Shooter/Turret/PID/kI", turretKi);
    Logger.recordOutput("Shooter/Turret/PID/kD", turretKd);
    Logger.recordOutput("Shooter/Turret/PID/IMaxAccum", turretIMaxAccumRot);
    Logger.recordOutput("Shooter/Turret/PID/IZone", turretIZoneRot);
    Logger.recordOutput("Shooter/Turret/PID/kS", turretKs);
    Logger.recordOutput("Shooter/Turret/PID/kV", turretKv);
    Logger.recordOutput("Shooter/Turret/PID/kA", turretKa);
  }

  @Override
  public void periodic() {
    logMotorStats("Shooter/Turret/Motor", turretMotor, false);
    logSendableValues();

    runTurret();
  }

  public void setPosition(double rotations) {
    turretSetpointRot = rotations;

    while (rotations <= turretMinAngleRot) {
      rotations += 1.0;
    }
    while (rotations >= turretMaxAngleRot) {
      rotations -= 1.0;
    }

    turretSetpointClampedRot = rotations;

    runTurret();
  }

  private void runTurret() {
    double currentPositionRot = getPositionRot();
    double arbFFVolts = turretArbFF.in(Volts);
    double totalFF = 0.0;

    if (turretSetpointClampedRot < currentPositionRot) {
      // Turret is moving clockwise
      totalFF =
          currentPositionRot < turretCableRetractorStartRot
              ? turretCableRetractorFFCW.in(Volts)
              : 0.0;
    } else {
      // Turret is moving counterclockwise
      totalFF =
          currentPositionRot < turretCableRetractorStartRot
              ? turretCableRetractorFFCCW.in(Volts)
              : 0.0;
    }

    totalFF += arbFFVolts;

    turretMotor
        .getClosedLoopController()
        .setSetpoint(
            turretSetpointClampedRot,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            totalFF,
            ArbFFUnits.kVoltage);
  }

  public void setOpenLoop(Voltage volts) {
    turretMotor.setVoltage(volts);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(turretSysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setOpenLoop(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(turretSysId.dynamic(direction));
  }

  public void setRelativeEncoderPosition(double rotations) {
    turretMotor.getEncoder().setPosition(rotations);
  }

  @AutoLogOutput(key = "Shooter/Turret/RelPosition", unit = "Rotations")
  public double getPositionRot() {
    return turretMotor.getEncoder().getPosition();
  }

  private void setPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    turretKp = p;
    turretKi = i;
    turretKd = d;
    config.closedLoop.pid(p, i, d);
    turretMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setPIDIntegralConstants(double iMaxAccumRot, double iZoneRot) {
    SparkFlexConfig config = new SparkFlexConfig();
    turretIMaxAccumRot = iMaxAccumRot;
    turretIZoneRot = iZoneRot;
    config.closedLoop.iMaxAccum(iMaxAccumRot).iZone(iZoneRot);
    turretMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setSVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    turretKs = s;
    turretKv = v;
    turretKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    turretMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
