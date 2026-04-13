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
          key + "/AbsPosition", Rotations.of(spark.getAbsoluteEncoder().getPosition()));
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

    Logger.recordOutput(key + "/Faults/Can", spark.getFaults().can);
    Logger.recordOutput(key + "/Faults/EscEeprom", spark.getFaults().escEeprom);
    Logger.recordOutput(key + "/Faults/Firmware", spark.getFaults().firmware);
    Logger.recordOutput(key + "/Faults/GateDriver", spark.getFaults().gateDriver);
    Logger.recordOutput(key + "/Faults/MotorType", spark.getFaults().motorType);
    Logger.recordOutput(key + "/Faults/Other", spark.getFaults().other);
    Logger.recordOutput(key + "/Faults/Sensor", spark.getFaults().sensor);
    Logger.recordOutput(key + "/Faults/Temperature", spark.getFaults().temperature);

    Logger.recordOutput(key + "/Warnings/Brownout", spark.getWarnings().brownout);
    Logger.recordOutput(key + "/Warnings/EscEeprom", spark.getWarnings().escEeprom);
    Logger.recordOutput(key + "/Warnings/ExtEeprom", spark.getWarnings().extEeprom);
    Logger.recordOutput(key + "/Warnings/HasReset", spark.getWarnings().hasReset);
    Logger.recordOutput(key + "/Warnings/Other", spark.getWarnings().other);
    Logger.recordOutput(key + "/Warnings/Overcurrent", spark.getWarnings().overcurrent);
    Logger.recordOutput(key + "/Warnings/Sensor", spark.getWarnings().sensor);
    Logger.recordOutput(key + "/Warnings/Stall", spark.getWarnings().stall);
  }
}
