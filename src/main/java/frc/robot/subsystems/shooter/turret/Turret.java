package frc.robot.subsystems.shooter.turret;

import static frc.robot.Constants.ShooterConstants.TurretConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {
  private SparkFlex turretMotor;
  private DigitalInput turretMagSensor;

  @Getter
  @AutoLogOutput(key = "Shooter/Turret/TurretSetpoint", unit = "rotations")
  private double turretSetpointRot = 0.0;

  private LoggedNetworkNumber turretP = new LoggedNetworkNumber("Tunable/Turret/kP", turretKp);
  private LoggedNetworkNumber turretI = new LoggedNetworkNumber("Tunable/Turret/kI", turretKi);
  private LoggedNetworkNumber turretD = new LoggedNetworkNumber("Tunable/Turret/kD", turretKd);

  private double magSensorStartPosition = 0.0;

  public Turret() {
    turretMotor = new SparkFlex(RobotMap.Shooter.turretMotorCanId, MotorType.kBrushless);

    SparkFlexConfig turretConfig = new SparkFlexConfig();
    turretConfig
        .smartCurrentLimit(turretCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(turretInverted);
    turretConfig.closedLoop.pid(turretKp, turretKi, turretKd);
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
  }

  public void setPosition(double rotations) {
    while (rotations <= turretMinAngleRot) {
      rotations += 1;
    }
    while (rotations >= turretMaxAngleRot) {
      rotations -= 1;
    }

    turretSetpointRot = rotations;
    turretMotor.getClosedLoopController().setSetpoint(rotations, ControlType.kPosition);
  }

  public void setOpenLoop(Voltage volts) {
    turretMotor.setVoltage(volts);
  }

  public void setRelativeEncoderPosition(Rotation2d position) {
    turretMotor.getEncoder().setPosition(position.getRotations());
  }

  public void rezeroFromAbsEncoder() {
    setRelativeEncoderPosition(getAbsPosition().minus(Rotation2d.kCCW_90deg));
  }

  @AutoLogOutput(key = "Shooter/Turret/MagSensor")
  private boolean isMagSensorTripped() {
    return turretMagSensor.get();
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(turretMotor.getEncoder().getPosition());
  }

  public Rotation2d getAbsPosition() {
    return Rotation2d.fromRotations(turretMotor.getAbsoluteEncoder().getPosition());
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
