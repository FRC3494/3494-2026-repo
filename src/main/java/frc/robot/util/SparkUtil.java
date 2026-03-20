// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }

  public static void logMotorStats(String key, SparkBase spark, boolean absoluteEncoder) {
    Logger.recordOutput(key + "/Position", Rotations.of(spark.getEncoder().getPosition()));
    Logger.recordOutput(key + "/Velocity", RPM.of(spark.getEncoder().getVelocity()));

    if (absoluteEncoder) {
      Logger.recordOutput(
          key + "/AbsPositionRad",
          Rotation2d.fromRotations(spark.getAbsoluteEncoder().getPosition()));
      Logger.recordOutput(key + "/AbsVelocity", RPM.of(spark.getAbsoluteEncoder().getVelocity()));
    }

    Logger.recordOutput(key + "/AppliedOutput", spark.getAppliedOutput());
    Logger.recordOutput(key + "/BusVoltage", Volts.of(spark.getBusVoltage()));
    Logger.recordOutput(
        key + "/AppliedVoltage", Volts.of(spark.getAppliedOutput() * spark.getBusVoltage()));
    Logger.recordOutput(key + "/Temp", Celsius.of(spark.getMotorTemperature()));
    Logger.recordOutput(key + "/Current", Amps.of(spark.getOutputCurrent()));

    Logger.recordOutput(key + "/Setpoint", spark.getClosedLoopController().getSetpoint());
    Logger.recordOutput(key + "/AtSetpoint", spark.getClosedLoopController().isAtSetpoint());
  }
}
