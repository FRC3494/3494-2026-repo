// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.util.QuadranglesUtil.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.RobotMap;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d zeroRotation;
  private Rotation2d relativeEncoderOffset;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final int moduleNumber;

  // @codescene(disable: "Complex Method", disable: "Large Method")
  public ModuleIOSpark(int module) {
    this.moduleNumber = module;

    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };
    driveSpark =
        new SparkFlex(
            switch (module) {
              case 0 -> RobotMap.Drive.frontLeftDriveCanId;
              case 1 -> RobotMap.Drive.frontRightDriveCanId;
              case 2 -> RobotMap.Drive.backLeftDriveCanId;
              case 3 -> RobotMap.Drive.backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSpark =
        new SparkFlex(
            switch (module) {
              case 0 -> RobotMap.Drive.frontLeftTurnCanId;
              case 1 -> RobotMap.Drive.frontRightTurnCanId;
              case 2 -> RobotMap.Drive.backLeftTurnCanId;
              case 3 -> RobotMap.Drive.backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnRelativeEncoder = turnSpark.getEncoder();
    turnAbsoluteEncoder =
        new AnalogInput(
            switch (module) {
              case 0 -> RobotMap.Drive.frontLeftAbsEncoderCanId;
              case 1 -> RobotMap.Drive.frontRightAbsEncoderCanId;
              case 2 -> RobotMap.Drive.backLeftAbsEncoderCanId;
              case 3 -> RobotMap.Drive.backRightAbsEncoderCanId;
              default -> 0;
            });
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new SparkFlexConfig();
    driveConfig
        .inverted(driveInverted[module])
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0)
        .openLoopRampRate(driveRampRate.in(Seconds))
        .closedLoopRampRate(driveRampRate.in(Seconds));
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, 0.0, driveKd);
    driveConfig.closedLoop.feedForward.sva(driveKs, driveKv, driveKa);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkFlexConfig();
    turnConfig
        .inverted(turnInverted[module])
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit[module])
        .voltageCompensation(12.0)
        .openLoopRampRate(turnRampRate.in(Seconds))
        .closedLoopRampRate(turnRampRate.in(Seconds));
    turnConfig
        .absoluteEncoder
        .inverted(turnRelEncoderInverted[module])
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig.encoder.quadratureAverageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            turnPIDMinInput * turnMotorReduction[module],
            turnPIDMaxInput * turnMotorReduction[module])
        .pid(turnKp, 0.0, turnKd);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    rezeroTurnEncoder();

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(turnSpark, () -> getTurnPosition().getRadians());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        () -> getTurnPosition().getRadians(),
        (value) -> inputs.turnPosition = Rotation2d.fromRadians(value));
    ifOk(
        turnSpark,
        turnRelativeEncoder::getVelocity,
        (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
    inputs.relativeRotationOffset = relativeEncoderOffset;
    inputs.rawRelativeTurnPosition = getRawRelativeTurnPosition();
    inputs.rawAbsoluteTurnPosition = getRawAbsoluteTurnPosition();
    inputs.absoluteTurnPosition = getAbsoluteTurnPosition();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRadians(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    logMotorStatsNotCoveredByIO("Drive/Module" + moduleNumber + "/DriveMotor", driveSpark);
    logMotorStatsNotCoveredByIO("Drive/Module" + moduleNumber + "/TurnMotor", turnSpark);
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveController.setSetpoint(velocityRadPerSec, ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        turnMotorReduction[moduleNumber]
            * MathUtil.inputModulus(
                rotation
                    .plus(relativeEncoderOffset)
                    .times(turnRelEncoderInverted[moduleNumber] ? -1 : 1)
                    .getRadians(),
                turnPIDMinInput,
                turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }

  @Override
  public void rezeroTurnEncoder() {
    relativeEncoderOffset = getRawRelativeTurnPosition().minus(getAbsoluteTurnPosition());
  }

  public Rotation2d getRawAbsoluteTurnPosition() {
    return new Rotation2d(
        turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * (2.0 * Math.PI));
  }

  public void setDrivePID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    driveKp = p;
    driveKi = i;
    driveKd = d;
    config.closedLoop.pid(p, i, d);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  public void setDriveSVA(double s, double v, double a) {
    SparkFlexConfig config = new SparkFlexConfig();
    driveKs = s;
    driveKv = v;
    driveKa = a;
    config.closedLoop.feedForward.sva(s, v, a);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  public void setTurnPID(double p, double i, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    turnKp = p;
    turnKi = i;
    turnKd = d;
    config.closedLoop.pid(p, i, d);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  public Rotation2d getAbsoluteTurnPosition() {
    return getRawAbsoluteTurnPosition()
        .minus(zeroRotation)
        .times(turnAbsEncoderInverted[moduleNumber] ? -1 : 1);
  }

  public Rotation2d getRawRelativeTurnPosition() {
    return Rotation2d.fromRadians(
            MathUtil.angleModulus(
                turnRelativeEncoder.getPosition() / turnMotorReduction[moduleNumber]))
        .times(turnRelEncoderInverted[moduleNumber] ? -1 : 1);
  }

  public Rotation2d getTurnPosition() {
    return getRawRelativeTurnPosition().minus(relativeEncoderOffset);
  }
}
