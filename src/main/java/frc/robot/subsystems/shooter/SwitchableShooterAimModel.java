package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Runtime-selectable wrapper around the linear and kinematics shooter aim models.
 *
 * <p>Operators can switch models live through NetworkTables while commands continue to use the
 * standard {@link ShooterAimModel} API.
 */
public class SwitchableShooterAimModel extends SubsystemBase implements ShooterAimModel {
  private final ShooterAimModel linearModel;
  private final ShooterAimModel kinematicsModel;
  private final LoggedNetworkBoolean useKinematicsModel =
      new LoggedNetworkBoolean("Tunable/Shooter/UseKinematicsModel", false);

  public SwitchableShooterAimModel(ShooterAimModel linearModel, ShooterAimModel kinematicsModel) {
    this.linearModel = linearModel;
    this.kinematicsModel = kinematicsModel;
  }

  private ShooterAimModel activeModel() {
    return isUsingKinematicsModel() ? kinematicsModel : linearModel;
  }

  @Override
  public void periodic() {
    ShooterAimModel.DebugState linearState = linearModel.getDebugState();
    ShooterAimModel.DebugState kinematicsState = kinematicsModel.getDebugState();

    Logger.recordOutput("ShooterAimModel/UseKinematicsModel", isUsingKinematicsModel());
    Logger.recordOutput(
        "ShooterAimModel/SelectedModel", isUsingKinematicsModel() ? "Kinematics" : "Linear");
        
    Logger.recordOutput(
        "ShooterAimModel/Compare/TargetPoseDelta",
        kinematicsState
            .targetPose()
            .getTranslation()
            .minus(linearState.targetPose().getTranslation()));
    Logger.recordOutput(
        "ShooterAimModel/Compare/VirtualTargetPoseDelta",
        kinematicsState
            .virtualTargetPose()
            .getTranslation()
            .minus(linearState.virtualTargetPose().getTranslation()));
    Logger.recordOutput(
        "ShooterAimModel/Compare/ShooterTranslationDelta",
        kinematicsState.shooterTranslation().minus(linearState.shooterTranslation()));
    Logger.recordOutput(
        "ShooterAimModel/Compare/TurretAngleDeltaRot",
        kinematicsState.turretAngleRot() - linearState.turretAngleRot());
    Logger.recordOutput(
        "ShooterAimModel/Compare/HoodAngleDeltaDeg",
        kinematicsState.hoodAngle().minus(linearState.hoodAngle()).getDegrees());
    Logger.recordOutput(
        "ShooterAimModel/Compare/FlywheelSpeedDeltaRPM",
        kinematicsState
            .flywheelSpeed()
            .minus(linearState.flywheelSpeed())
            .in(edu.wpi.first.units.Units.RPM));
    Logger.recordOutput(
        "ShooterAimModel/Compare/TurretFFDeltaVolts",
        kinematicsState
            .turretFF()
            .minus(linearState.turretFF())
            .in(edu.wpi.first.units.Units.Volts));
  }

  @Override
  @AutoLogOutput(key = "ShooterAimModel/TurretAngleRot")
  public double getTurretAngleRot() {
    return activeModel().getTurretAngleRot();
  }

  @Override
  @AutoLogOutput(key = "ShooterAimModel/TurretFFVolts")
  public Voltage getTurretFF() {
    return activeModel().getTurretFF();
  }

  @Override
  @AutoLogOutput(key = "ShooterAimModel/HoodAngle")
  public Rotation2d getHoodAngle() {
    return activeModel().getHoodAngle();
  }

  @Override
  @AutoLogOutput(key = "ShooterAimModel/FlywheelSpeed")
  public AngularVelocity getFlywheelSpeed() {
    return activeModel().getFlywheelSpeed();
  }

  @Override
  public AngularVelocity applyFlywheelTrim(AngularVelocity baseSpeed) {
    return activeModel().applyFlywheelTrim(baseSpeed);
  }

  @Override
  public Rotation2d applyHoodTrim(Rotation2d baseAngle) {
    return activeModel().applyHoodTrim(baseAngle);
  }

  @Override
  public double getTurretTrimRot() {
    return activeModel().getTurretTrimRot();
  }

  @Override
  public void setTurretTrim(double trimRot) {
    linearModel.setTurretTrim(trimRot);
    kinematicsModel.setTurretTrim(trimRot);
  }

  @Override
  public Rotation2d getHoodTrim() {
    return activeModel().getHoodTrim();
  }

  @Override
  public void setHoodTrim(Rotation2d trim) {
    linearModel.setHoodTrim(trim);
    kinematicsModel.setHoodTrim(trim);
  }

  @Override
  public AngularVelocity getFlywheelTrim() {
    return activeModel().getFlywheelTrim();
  }

  @Override
  public void setFlywheelTrim(AngularVelocity trim) {
    linearModel.setFlywheelTrim(trim);
    kinematicsModel.setFlywheelTrim(trim);
  }

  @Override
  public Distance getDistanceTrim() {
    return activeModel().getDistanceTrim();
  }

  @Override
  public void setDistanceTrim(Distance trim) {
    linearModel.setDistanceTrim(trim);
    kinematicsModel.setDistanceTrim(trim);
  }

  @Override
  public Distance getXTrim() {
    return activeModel().getXTrim();
  }

  @Override
  public void setXTrim(Distance trim) {
    linearModel.setXTrim(trim);
    kinematicsModel.setXTrim(trim);
  }

  @Override
  public Distance getYTrim() {
    return activeModel().getYTrim();
  }

  @Override
  public void setYTrim(Distance trim) {
    linearModel.setYTrim(trim);
    kinematicsModel.setYTrim(trim);
  }

  public boolean isUsingKinematicsModel() {
    return useKinematicsModel.get();
  }

  public void setUseKinematicsModel(boolean useKinematics) {
    useKinematicsModel.set(useKinematics);
  }
}
