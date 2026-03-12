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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {
  private SparkFlex turretMotor;
  private DigitalInput turretMagSensor;

  @Getter
  @AutoLogOutput(key = "Shooter/Turret/TurretSetpoint", unit = "rotations")
  private double turretSetpointRot = 0.0;

  @Getter
  @AutoLogOutput(key = "Shooter/Turret/TurretSetpointClamped", unit = "rotations")
  private double turretSetpointClampedRot = 0.0;

  private LoggedNetworkNumber turretP = new LoggedNetworkNumber("Tunable/Turret/kP", turretKp);
  private LoggedNetworkNumber turretI = new LoggedNetworkNumber("Tunable/Turret/kI", turretKi);
  private LoggedNetworkNumber turretD = new LoggedNetworkNumber("Tunable/Turret/kD", turretKd);

  private LoggedNetworkNumber turretCableRetractorFFVolts =
      new LoggedNetworkNumber("Tunable/Turret/CableRetractorFF", turretCableRetractorFF.in(Volts));

  private double magSensorStartPosition = 0.0;

  SysIdRoutine turretSysId;

  public Turret() {
    turretMotor = new SparkFlex(RobotMap.Shooter.turretMotorCanId, MotorType.kBrushless);

    SparkFlexConfig turretConfig = new SparkFlexConfig();
    turretConfig
        .smartCurrentLimit(turretCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(turretInverted);
    turretConfig
        .closedLoop
        .pid(turretKp, turretKi, turretKd)
        .iMaxAccum(turretIMaxAccum)
        .iZone(turretIZone)
        .allowedClosedLoopError(turretPositionTolerance, ClosedLoopSlot.kSlot0);
    turretConfig.closedLoop.feedForward.sva(turretKs, turretKv, turretKa);
    turretConfig
        .encoder
        .positionConversionFactor(turretGearRatio)
        .velocityConversionFactor(turretGearRatio);
    turretConfig
        .absoluteEncoder
        .zeroOffset(turretAbsEncoderOffset)
        .positionConversionFactor(turretAbsEncoderGearRatio)
        .velocityConversionFactor(turretAbsEncoderGearRatio);
    turretMotor.configure(
        turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretMagSensor = new DigitalInput(RobotMap.Shooter.turretMagSensorDIO);

    turretSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(0.1),
                Volts.of(1),
                null,
                (state) -> Logger.recordOutput("Shooter/Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setOpenLoop(voltage), null, this));

    // new Trigger(this::isMagSensorTripped)
    //     .and(ShooterOI.rezeroTurret())
    //     .onTrue(
    //         runOnce(
    //                 () -> {
    //                   magSensorStartPosition = getRawAbsPosition();
    //                 })
    //             .ignoringDisable(true))
    //     .onFalse(
    //         runOnce(
    //                 () -> {
    //                   double magSensorEndPosition = getRawAbsPosition();
    //                   double magSensorPosition =
    //                       (magSensorStartPosition + magSensorEndPosition) / 2;

    //                   double absEncoderOffset =
    //                       MathUtil.inputModulus(magSensorPosition - turretAbsEncoderOffset, 0,
    // 1);

    //                   NumberFormat formatter = new DecimalFormat("#0.00000000");
    //                   System.out.println("********** Turret Zeroing Results **********");
    //                   System.out.println(
    //                       "\tAbsolute Encoder Offset: " + formatter.format(absEncoderOffset));
    //                 })
    //             .ignoringDisable(true));

    rezeroFromAbsEncoder();
  }

  @Override
  public void periodic() {
    logMotorStats("Shooter/Turret/Motor", turretMotor, true);

    boolean pidChanged =
        turretP.get() != turretKp || turretI.get() != turretKi || turretD.get() != turretKd;
    if (pidChanged) {
      setPID(turretP.get(), turretI.get(), turretD.get());
    }

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
  }

  private void runTurret() {
    turretMotor
        .getClosedLoopController()
        .setSetpoint(
            turretSetpointClampedRot,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            getPositionRot() < turretCableRetractorStart ? turretCableRetractorFFVolts.get() : 0.0,
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

  public void rezeroFromAbsEncoder() {
    setRelativeEncoderPosition(getAbsPositionRot() - Units.degreesToRotations(90));
  }

  @AutoLogOutput(key = "Shooter/Turret/MagSensor")
  private boolean isMagSensorTripped() {
    return turretMagSensor.get();
  }

  @AutoLogOutput(key = "Shooter/Turret/RelPosition", unit = "Rotations")
  public double getPositionRot() {
    return turretMotor.getEncoder().getPosition();
  }

  public double getAbsPositionRot() {
    return turretMotor.getAbsoluteEncoder().getPosition();
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
}
