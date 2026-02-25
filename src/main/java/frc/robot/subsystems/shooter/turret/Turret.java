package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ShooterConstants.TurretConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotMap;
import frc.robot.OI.ShooterOI;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Turret extends SubsystemBase {
  private SparkFlex turretMotor;
  private DigitalInput turretMagSensor;

  @Getter @AutoLogOutput private Rotation2d turretSetpoint = Rotation2d.kZero;

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
        turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    turretMagSensor = new DigitalInput(RobotMap.Shooter.turretMagSensorDIO);

    new Trigger(this::isMagSensorTripped)
        .and(ShooterOI.rezeroTurret())
        .onTrue(
            runOnce(
                    () -> {
                      magSensorStartPosition = getRawAbsPosition();
                    })
                .ignoringDisable(true))
        .onFalse(
            runOnce(
                    () -> {
                      double magSensorEndPosition = getRawAbsPosition();
                      double magSensorPosition =
                          (magSensorStartPosition + magSensorEndPosition) / 2;

                      double absEncoderOffset =
                          MathUtil.inputModulus(magSensorPosition - turretAbsEncoderOffset, 0, 1);

                      NumberFormat formatter = new DecimalFormat("#0.00000000");
                      System.out.println("********** Turret Zeroing Results **********");
                      System.out.println(
                          "\tAbsolute Encoder Offset: " + formatter.format(absEncoderOffset));
                    })
                .ignoringDisable(true));
  }

  @Override
  public void periodic() {
    logMotorStats("Turret/Motor", turretMotor, false);
    logTurretInfo();
  }

  public void logTurretInfo() {
    Logger.recordOutput("Turret/MagSensorTripped", isMagSensorTripped());
  }

  public void setPosition(Rotation2d setpoint) {
    turretSetpoint = setpoint;
    turretMotor
        .getClosedLoopController()
        .setSetpoint(setpoint.getRotations(), ControlType.kMAXMotionPositionControl);
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

  private double getRawAbsPosition() {
    return turretMotor.getAbsoluteEncoder().getPosition() / turretAbsEncoderGearRatio;
  }


}
