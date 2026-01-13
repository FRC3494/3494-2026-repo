package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  private static CommandXboxController primaryController = new CommandXboxController(0);
  private static Joystick leftButtonBoard = new Joystick(1);
  private static Joystick rightButtonBoard = new Joystick(2);

  public static void update() {
    eventLoop.poll();
  }

  public static class Drive {
    public static double joystickDriveX() {
      return -primaryController.getLeftY();
    }

    public static double joystickDriveY() {
      return -primaryController.getLeftX();
    }

    public static double joystickDriveOmega() {
      return -primaryController.getRightX();
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
  }
}
