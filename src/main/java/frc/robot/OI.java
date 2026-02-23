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

    public static Trigger autoAlignClimb() {
      return new Trigger(() -> false);
      // return primaryController.y(eventLoop);
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

    public static Trigger lockTo45() {
      // return primaryController.a(eventLoop);
      return new Trigger(() -> false);
    }

    public static Trigger resetYawPigeon() {
      // return primaryController.b(eventLoop);
      return new Trigger(() -> false);
    }
  }

  public static final class ClimberOI {
    public static Trigger climberDown() {
      return primaryController.a(eventLoop);
    }

    public static Trigger climberUp() {
      return primaryController.y(eventLoop);
    }
  }

  public static final class IntakeOI {
    public static Trigger intake() {
      return primaryController.rightTrigger(controllerTriggerDeadband, eventLoop);
    }

    public static Trigger outtake() {
      return primaryController.x(eventLoop);
    }
  }

  public static final class ShooterOI {
    public static Trigger setHubShot() {
      return rightButtonBoard.button(1, eventLoop).castTo(Trigger::new);
    }

    public static Trigger setTrenchShot() {
      return rightButtonBoard.button(2, eventLoop).castTo(Trigger::new);
    }

    public static Trigger setOutpostShot() {
      return rightButtonBoard.button(3, eventLoop).castTo(Trigger::new);
    }

    public static Trigger setNeutralZoneShot() {
      return rightButtonBoard.button(4, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shoot() {
      return primaryController.leftTrigger(controllerTriggerDeadband, eventLoop);
    }

    public static Trigger runSpindexer() {
      return primaryController.b(eventLoop);
    }

    public static Trigger runFeeder() {
      return primaryController.leftBumper(eventLoop);
    }

    public static Trigger increaseHood() {
      return primaryController.povUp();
    }

    public static Trigger decreaseHood() {
      return primaryController.povDown();
    }

    public static Trigger runFlywheel() {
      return primaryController.rightBumper(eventLoop);
    }

    public static Trigger rezeroTurret() {
      // return primaryController.povRight();
      return new Trigger(() -> false);
    }
  }
}
