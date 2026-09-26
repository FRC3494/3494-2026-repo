// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final CANBus kCANBus;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOTalonFX(int module) {
    kCANBus = new CANBus("canivore", "./logs/example.hoot");

    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> RobotMap.Drive.frontLeftDriveCanId;
              case 1 -> RobotMap.Drive.frontRightDriveCanId;
              case 2 -> RobotMap.Drive.backLeftDriveCanId;
              case 3 -> RobotMap.Drive.backRightDriveCanId;
              default -> throw new IllegalArgumentException("Invalid module index");
            },
            kCANBus);
    turnTalon =
        new TalonFX(
            switch (module) {
              case 0 -> RobotMap.Drive.frontLeftTurnCanId;
              case 1 -> RobotMap.Drive.frontRightTurnCanId;
              case 2 -> RobotMap.Drive.backLeftTurnCanId;
              case 3 -> RobotMap.Drive.backRightTurnCanId;
              default -> throw new IllegalArgumentException("Invalid module index");
            },
            kCANBus);
    cancoder =
        new CANcoder(
            switch (module) {
              case 0 -> RobotMap.Drive.frontLeftAbsEncoderCanId;
              case 1 -> RobotMap.Drive.frontRightAbsEncoderCanId;
              case 2 -> RobotMap.Drive.backLeftAbsEncoderCanId;
              case 3 -> RobotMap.Drive.backRightAbsEncoderCanId;
              default -> throw new IllegalArgumentException("Invalid module index");
            },
            kCANBus);

    var driveConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(driveMotorCurrentLimit))
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(driveMotorCurrentLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        driveInverted[module]
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withSlot0(
                new Slot0Configs()
                    .withKP(driveKp)
                    .withKI(driveKi)
                    .withKD(driveKd)
                    .withKS(driveKs)
                    .withKV(driveKv)
                    .withKA(driveKa))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(driveMotorReduction))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(driveMotorCurrentLimit)
                    .withPeakReverseTorqueCurrent(-driveMotorCurrentLimit));

    // Configure drive motor
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    var turnConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(driveMotorCurrentLimit))
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        turnInverted[module]
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withSlot0(
                new Slot0Configs()
                    .withKP(turnKp)
                    .withKI(turnKi)
                    .withKD(turnKd)
                    .withKS(turnKs)
                    .withKV(turnKv)
                    .withKA(turnKa))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(1.0)
                    .withFeedbackRemoteSensorID(
                        switch (module) {
                          case 0 -> RobotMap.Drive.frontLeftAbsEncoderCanId;
                          case 1 -> RobotMap.Drive.frontRightAbsEncoderCanId;
                          case 2 -> RobotMap.Drive.backLeftAbsEncoderCanId;
                          case 3 -> RobotMap.Drive.backRightAbsEncoderCanId;
                          default -> throw new IllegalArgumentException("Invalid module index");
                        })
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withRotorToSensorRatio(turnMotorReduction[module]))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(turnMotorCurrentLimit[module])
                    .withPeakReverseTorqueCurrent(-turnMotorCurrentLimit[module]))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));

    // Configure turn motor
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
    tryUntilOk(5, () -> turnTalon.setPosition(0.0, 0.25));

    var cancoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withMagnetOffset(
                        switch (module) {
                          case 0 -> frontLeftZeroRotation.getRotations();
                          case 1 -> frontRightZeroRotation.getRotations();
                          case 2 -> backLeftZeroRotation.getRotations();
                          case 3 -> backRightZeroRotation.getRotations();
                          default -> throw new IllegalArgumentException("Invalid module index");
                        })
                    .withSensorDirection(
                        turnAbsEncoderInverted[module]
                            ? SensorDirectionValue.Clockwise_Positive
                            : SensorDirectionValue.CounterClockwise_Positive));
    cancoder.getConfigurator().apply(cancoderConfig);

    // Create timestamp queue
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(() -> drivePosition.asSupplier().get().in(Radians));
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        SparkOdometryThread.getInstance()
            .registerSignal(() -> turnPosition.asSupplier().get().in(Radians));
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.odometryFrequency, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
  }

  public void rezeroTurnEncoder() {}

  public void setDrivePID(double p, double i, double d) {
    
  }

  public void setDriveSVA(double s, double v, double a) {}

  public void setTurnPID(double p, double i, double d) {}
  public void setTurnSVA(double s, double v, double a) {}

  public void setDriveRampRate(Time rate) {}

  public void setTurnRampRate(Time rate) {}
}
