package frc.robot.subsystems.drive.autoalign;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.util.QuadranglesUtil.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommand extends Command {
  private final Drive drive;

  private final Pose2d targetPose;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController headingController;

  private final Distance xTolerance;
  private final Distance yTolerance;
  private final Rotation2d angularTolerance;

  public AutoAlignCommand(
      Pose2d targetPose,
      Drive drive,
      Distance xTolerance,
      Distance yTolerance,
      Rotation2d angularTolerance) {
    this.targetPose = toAlliancePose(targetPose);
    this.drive = drive;

    addRequirements(drive);

    xController = new PIDController(autoAlignLinearKp, autoAlignLinearKi, autoAlignLinearKd);
    yController = new PIDController(autoAlignLinearKp, autoAlignLinearKi, autoAlignLinearKd);
    headingController =
        new PIDController(autoAlignAngularKp, autoAlignAngularKi, autoAlignAngularKd);

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    this.xTolerance = xTolerance;
    this.yTolerance = yTolerance;
    this.angularTolerance = angularTolerance;
  }

  public AutoAlignCommand(
      Pose2d targetPose, Drive drive, Distance linearTolerance, Rotation2d angularTolerance) {
    this(targetPose, drive, linearTolerance, linearTolerance, angularTolerance);
  }

  public AutoAlignCommand(Pose2d targetPose, Drive drive) {
    this(targetPose, drive, autoAlignLinearTolerance, autoAlignAngularTolerance);
  }

  @Override
  public void initialize() {
    drive.setAutoAligning(true);
  }

  @Override
  public void execute() {
    Logger.recordOutput("Drive/AutoAlign/TargetPose", targetPose);

    Pose2d currentPose = drive.getPose();

    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double omega =
        headingController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, drive.getRotation());
    ChassisSpeeds adjustedSpeeds = chassisSpeeds.minus(drive.getChassisSpeeds());

    drive.runVelocity(adjustedSpeeds);

    Logger.recordOutput(
        "Drive/AutoAlign/VelocitySetpoint",
        new Pose2d(
            currentPose.getX() + xSpeed,
            currentPose.getY() + ySpeed,
            Rotation2d.fromRadians(currentPose.getRotation().getRadians() + omega)));
    Logger.recordOutput("Drive/AutoAlign/XError", Meters.of(xController.getError()));
    Logger.recordOutput("Drive/AutoAlign/YError", Meters.of(yController.getError()));
    Logger.recordOutput(
        "Drive/AutoAlign/HeadingError", Rotation2d.fromRadians(headingController.getError()));
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drive.getPose();

    boolean xAtSetpoint = currentPose.getMeasureX().isNear(targetPose.getMeasureX(), xTolerance);
    Logger.recordOutput("Drive/AutoAlign/XAtSetpoint", xAtSetpoint);
    boolean yAtSetpoint = currentPose.getMeasureY().isNear(targetPose.getMeasureY(), yTolerance);
    Logger.recordOutput("Drive/AutoAlign/YAtSetpoint", yAtSetpoint);
    boolean headingAtSetpoint =
        (currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians())
            <= angularTolerance.getRadians();
    Logger.recordOutput("Drive/AutoAlign/HeadingAtSetpoint", headingAtSetpoint);

    return xAtSetpoint && yAtSetpoint && headingAtSetpoint;
  }

  public static Command alignSequence(Drive drive, Pose2d... poses) {
    return sequence(
        Arrays.stream(poses)
            .map((Pose2d pose) -> new AutoAlignCommand(pose, drive))
            .toArray(AutoAlignCommand[]::new));
  }

  public static Command alignSequence(
      Drive drive,
      Distance xTolerance,
      Distance yTolerance,
      Rotation2d angularTolerance,
      Pose2d... poses) {
    return sequence(
        Arrays.stream(poses)
            .map(
                (Pose2d pose) ->
                    new AutoAlignCommand(pose, drive, xTolerance, yTolerance, angularTolerance))
            .toArray(AutoAlignCommand[]::new));
  }

  public static Command alignSequence(
      Drive drive, Distance linearTolerance, Rotation2d angularTolerance, Pose2d... poses) {
    return sequence(
        Arrays.stream(poses)
            .map(
                (Pose2d pose) ->
                    new AutoAlignCommand(pose, drive, linearTolerance, angularTolerance))
            .toArray(AutoAlignCommand[]::new));
  }

  /** Wrapper for `alignSequence` that makes it a DeferredCommand. */
  public static Command alignSequenceDeferred(Drive drive, Pose2d... poses) {
    return defer(() -> alignSequence(drive, poses), Set.of(drive));
  }

  public static Command alignSequenceDeferred(
      Drive drive,
      Distance xTolerance,
      Distance yTolerance,
      Rotation2d angularTolerance,
      Pose2d... poses) {
    return defer(
        () -> alignSequence(drive, xTolerance, yTolerance, angularTolerance, poses), Set.of(drive));
  }

  public static Command alignSequenceDeferred(
      Drive drive, Distance linearTolerance, Rotation2d angularTolerance, Pose2d... poses) {
    return defer(
        () -> alignSequence(drive, linearTolerance, angularTolerance, poses), Set.of(drive));
  }
}
