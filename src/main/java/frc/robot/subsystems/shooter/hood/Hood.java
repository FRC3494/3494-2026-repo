package frc.robot.subsystems.shooter.hood;

import static frc.robot.Constants.HoodConstants.*;
import static frc.robot.util.SparkUtil.logMotorStats;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hood extends SubsystemBase {
  private SparkMax hoodMotor;

  @AutoLogOutput private Rotation2d hoodSetpoint = Rotation2d.kZero;

  public Hood() {
    hoodMotor = new SparkMax(RobotMap.hoodMotorCanId, MotorType.kBrushless);

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig
        .smartCurrentLimit(hoodCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(hoodInverted);
  }

  @Override
  public void periodic() {
    logMotorStats("Hood/Motor", hoodMotor, false);
  }
}
