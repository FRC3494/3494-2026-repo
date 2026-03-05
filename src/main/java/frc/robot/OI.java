package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public final class OI {
  private static EventLoop eventLoop = new EventLoop();
  private static CommandXboxController primaryController =
      new CommandXboxController(primaryControllerPort);
  private static Joystick leftButtonBoard = new Joystick(leftButtonBoardPort);
  private static Joystick rightButtonBoard = new Joystick(rightButtonBoardPort);

  public static void update() {
    eventLoop.poll();

    Logger.recordOutput("OI/JoystickDriveX", DriveOI.joystickDriveX());
    Logger.recordOutput("OI/JoystickDriveY", DriveOI.joystickDriveY());
    Logger.recordOutput("OI/JoystickDriveOmega", DriveOI.joystickDriveOmega());
  }

  public static final class ClimberOI {
    public static Trigger climberUp() {
      return primaryController.y(eventLoop);
    }

    public static Trigger climberDown() {
      return primaryController.a(eventLoop);
    }

    public static Trigger rezeroClimber() {
      return leftButtonBoard.button(1, eventLoop).castTo(Trigger::new);
    }

    public static Trigger climberManualUp() {
      return rightButtonBoard
          .axisGreaterThan(1, controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger climberManualDown() {
      return rightButtonBoard
          .axisLessThan(1, -controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }
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

    public static Trigger lockTo45() {
      // return primaryController.a(eventLoop);
      return new Trigger(() -> false);
    }

    public static Trigger autoAlignClimb() {
      return primaryController.povLeft();
    }
  }

  public static final class HopperOI {
    public static Trigger runSpindexer() {
      return primaryController.b(eventLoop);
    }

    public static Trigger runKicker() {
      return primaryController.leftBumper(eventLoop);
    }
  }

  public static final class IntakeOI {
    public static Trigger intake() {
      return primaryController.rightTrigger(controllerTriggerDeadband, eventLoop);
    }

    public static Trigger outtake() {
      return primaryController.x(eventLoop);
    }

    public static Trigger toggleIntake() {
      return rightButtonBoard.button(10, eventLoop).castTo(Trigger::new);
    }
  }

  public static final class ShooterOI {
    public static Trigger shoot() {
      return primaryController.leftTrigger(controllerTriggerDeadband, eventLoop);
    }

    public static Trigger shootClose() {
      return rightButtonBoard.button(1, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootMedium() {
      return rightButtonBoard.button(2, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootFar() {
      return rightButtonBoard.button(3, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootNeutralZone() {
      return rightButtonBoard.button(4, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootDashboard() {
      return leftButtonBoard.button(6, eventLoop).castTo(Trigger::new);
    }

    public static final class FlywheelOI {
      public static Trigger runFlywheel() {
        return primaryController.rightBumper(eventLoop);
      }
    }

    public static final class HoodOI {
      public static Trigger hoodManualUp() {
        return primaryController.povUp();
      }

      public static Trigger hoodManualDown() {
        return primaryController.povDown();
      }

      public static Trigger rezeroHood() {
        return leftButtonBoard.button(2, eventLoop).castTo(Trigger::new);
      }
    }

    public static final class TurretOI {
      public static Trigger turretManualCCW() {
        return leftButtonBoard
            .axisLessThan(0, -controllerStickDeadband, eventLoop)
            .castTo(Trigger::new);
      }

      public static Trigger turretManualCW() {
        return leftButtonBoard
            .axisGreaterThan(0, controllerStickDeadband, eventLoop)
            .castTo(Trigger::new);
      }

      public static Trigger rezeroTurret() {
        return leftButtonBoard.button(3, eventLoop).castTo(Trigger::new);
      }

      public static Trigger enableAutoTurret() {
        return leftButtonBoard.button(9, eventLoop).castTo(Trigger::new);
      }

      public static Trigger setTurretEncoderTo0() {
        return leftButtonBoard.button(10, eventLoop).castTo(Trigger::new);
      }

      public static Trigger turretTo180() {
        return leftButtonBoard.button(9, eventLoop).castTo(Trigger::new);
      }

      public static Trigger turretTo90() {
        return leftButtonBoard.button(8, eventLoop).castTo(Trigger::new);
      }
    }
  }
}
