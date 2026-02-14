package frc.robot.subsystems.drive.autoalign;

import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.QuadranglesUtil;
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

  public AutoAlignCommand(Pose2d targetPose, Drive drive) {
    this.targetPose = QuadranglesUtil.toAlliancePose(targetPose);
    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.setAutoAligning(true);
  }

  @Override
  public void execute() {
    Logger.recordOutput("Drive/AutoAlign/TargetPose", targetPose);

    Pose2d currentPose = drive.getPose();

    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.calculate(currentPose.getX(), targetPose.getX()),
            yController.calculate(currentPose.getY(), targetPose.getY()),
            headingController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()),
            drive.getRotation());

    drive.runVelocity(chassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
