// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElasticTab;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.Elastic;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;
  private final Hopper hopper;

  // Dashboard inputs
  private final AutoChooser autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    flywheel = new Flywheel();
    hood = new Hood();
    turret = new Turret();
    hopper = new Hopper();

    RobotModeTriggers.autonomous()
        .onTrue(runOnce(() -> Elastic.selectTab(ElasticTab.Autonomous.toString())));
    RobotModeTriggers.teleop()
        .onTrue(runOnce(() -> Elastic.selectTab(ElasticTab.Teleoperated.toString())));

    // Set up auto routines
    autoChooser = new AutoChooser();
    configureAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureAutos() {
    // Set up SysId routines
    autoChooser.addCmd(
        "Drive Wheel Radius Rotational Characterization",
        () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "Drive Wheel Radius Linear Characterization",
        () -> DriveCommands.linearWheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "Drive Simple FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Forward)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Reverse)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ==================== DRIVE ====================
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            OI.Drive::joystickDriveX,
            OI.Drive::joystickDriveY,
            OI.Drive::joystickDriveOmega));

    // Lock to 0° when A button is held
    OI.Drive.lockToForward()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, OI.Drive::joystickDriveX, OI.Drive::joystickDriveY, () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    OI.Drive.stopWithX().onTrue(runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Back button is pressed
    OI.Drive.resetYaw().onTrue(runOnce(drive::resetYaw).ignoringDisable(true));

    // Rezero swerve turn relative encoders off of absolute encoders
    OI.Drive.rezeroSwerveTurnEncoders()
        .onTrue(runOnce(drive::rezeroTurnEncoders).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
