package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  private static CommandXboxController primaryController =
      new CommandXboxController(primaryControllerPort);
  private static Joystick leftButtonBoard = new Joystick(leftButtonBoardPort);
  private static Joystick rightButtonBoard = new Joystick(rightButtonBoardPort);

  public static void update() {
    eventLoop.poll();
  }

  public static final class DriveOI {
    public static double joystickDriveX() {
      return MathUtil.applyDeadband(-primaryController.getLeftY(), controllerStickDeadband);
    }

    public static double joystickDriveY() {
      return MathUtil.applyDeadband(-primaryController.getLeftX(), controllerStickDeadband);
    }

    public static double joystickDriveOmega() {
      return MathUtil.applyDeadband(-primaryController.getRightX(), controllerStickDeadband);
    }

    public static Trigger resetYaw() {
      return primaryController.back(eventLoop);
    }

    public static Trigger rezeroSwerveTurnEncoders() {
      return primaryController.start(eventLoop);
    }

    public static Trigger stopWithX() {
      return primaryController.x(eventLoop);
    }

    public static Trigger lockToForward() {
      return primaryController.a(eventLoop);
    }

    public static Trigger resetYawPigeon() {
      return primaryController.b(eventLoop);
    }
  }
}
