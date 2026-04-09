package frc.robot.subsystems.drive.autoalign;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.QuadranglesUtil;
import java.util.Arrays;
import java.util.Set;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignCommand extends Command {
  private final Drive drive;

  @Getter @AutoLogOutput private final Pose2d targetPose;

  private final PIDController xController =
      new PIDController(autoAlignLinearKp, autoAlignLinearKi, autoAlignLinearKd);
  private final PIDController yController =
      new PIDController(autoAlignLinearKp, autoAlignLinearKi, autoAlignLinearKd);
  private final PIDController headingController =
      new PIDController(autoAlignAngularKp, autoAlignAngularKi, autoAlignAngularKd);

  public AutoAlignCommand(
      Pose2d targetPose, Drive drive, Distance linearTolerance, Rotation2d angularTolerance) {
    this.targetPose = QuadranglesUtil.toAlliancePose(targetPose);
    this.drive = drive;

    addRequirements(drive);

    xController.setTolerance(linearTolerance.in(Meters));
    yController.setTolerance(linearTolerance.in(Meters));
    headingController.setTolerance(angularTolerance.getRadians());
    headingController.enableContinuousInput(-Math.PI, Math.PI);
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

    double xSpeed =
        !xController.atSetpoint()
            ? xController.calculate(currentPose.getX(), targetPose.getX())
            : 0;
    double ySpeed =
        !yController.atSetpoint()
            ? yController.calculate(currentPose.getY(), targetPose.getY())
            : 0;
    double omega =
        !headingController.atSetpoint()
            ? headingController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians())
            : 0;

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, drive.getRotation());

    drive.runVelocity(chassisSpeeds);

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
    return xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint();
  }

  public static Command alignSequence(Drive drive, Pose2d... poses) {
    return sequence(
        Arrays.stream(poses)
            .map((Pose2d pose) -> new AutoAlignCommand(pose, drive))
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
      Drive drive, Distance linearTolerance, Rotation2d angularTolerance, Pose2d... poses) {
    return defer(
        () -> alignSequence(drive, linearTolerance, angularTolerance, poses), Set.of(drive));
  }
}
