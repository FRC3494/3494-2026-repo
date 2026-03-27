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

  public static Trigger rezeroMechanisms() {
    // Just hit individual buttons lol
    return new Trigger(() -> false);
  }

  public static final class ClimberOI {
    public static Trigger climberUp() {
      return rightButtonBoard.button(6, eventLoop).castTo(Trigger::new);
      // return primaryController.y(eventLoop).or(rightButtonBoard.button(6, eventLoop));
    }

    public static Trigger climberDown() {
      return new Trigger(() -> false);
      // return primaryController.a(eventLoop);
    }

    public static Trigger climberAllTheWayDown() {
      return rightButtonBoard.button(5, eventLoop).castTo(Trigger::new);
    }

    public static Trigger actuallyClimb() {
      return rightButtonBoard.button(7, eventLoop).castTo(Trigger::new);
    }

    public static Trigger rezeroClimber() {
      return leftButtonBoard.button(1, eventLoop).castTo(Trigger::new);
    }

    public static Trigger climberManualUp() {
      // Just use climber rezero
      return new Trigger(() -> false);
    }

    public static Trigger climberManualDown() {
      return new Trigger(() -> false);
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
      return primaryController.rightBumper(eventLoop);
    }

    public static Trigger lockTo45() {
      // return primaryController.a(eventLoop);
      return new Trigger(() -> false);
    }

    public static Trigger autoAlignClimb() {
      return primaryController.leftBumper(eventLoop);
    }
  }

  public static final class HopperOI {
    public static Trigger unjamSpindexer() {
      return new Trigger(() -> false);
      // return leftButtonBoard.button(7, eventLoop).castTo(Trigger::new);
    }

    public static Trigger spindexerBackwards() {
      return leftButtonBoard.button(7, eventLoop).castTo(Trigger::new);
    }

    public static Trigger runKicker() {
      return new Trigger(() -> false);
      // return primaryController.leftBumper(eventLoop);
    }

    public static Trigger kickerBackwards() {
      return rightButtonBoard.button(4, eventLoop).castTo(Trigger::new);
    }

    public static Trigger jiggleRobot() {
      return primaryController.povLeft();
    }
  }

  public static final class IntakeOI {
    public static Trigger intake() {
      return primaryController
          .rightTrigger(controllerTriggerDeadband, eventLoop)
          .or(rightButtonBoard.button(10, eventLoop));
    }

    public static Trigger outtake() {
      return primaryController.x(eventLoop);
    }

    public static Trigger toggleIntake() {
      return new Trigger(() -> false);
      // return rightButtonBoard.button(10, eventLoop).castTo(Trigger::new);
    }

    public static Trigger rezeroIntakeUppyDowny() {
      return new Trigger(() -> false);
    }

    public static Trigger jostleIntake() {
      return primaryController.b(eventLoop);
    }

    // L3
    public static Trigger raiseIntake() {
      return primaryController.button(9, eventLoop);
    }

    // R3
    public static Trigger lowerIntake() {
      return primaryController.button(10, eventLoop);
    }
  }

  public static final class ShooterOI {
    public static Trigger shoot() {
      return primaryController
          .leftTrigger(controllerTriggerDeadband, eventLoop)
          .or(rightButtonBoard.button(8, eventLoop));
    }

    public static Trigger shootClose() {
      return rightButtonBoard.button(1, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootCloseWithTrim() {
      return new Trigger(() -> false);
      // return rightButtonBoard.button(5, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootMedium() {
      return rightButtonBoard.button(2, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootMediumWithTrim() {
      return new Trigger(() -> false);
      // return rightButtonBoard.button(6, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootFar() {
      return rightButtonBoard.button(3, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootFarWithTrim() {
      return new Trigger(() -> false);
      // return rightButtonBoard.button(7, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootNeutralZone() {
      return new Trigger(() -> false);
      // return rightButtonBoard.button(4, eventLoop).castTo(Trigger::new);
    }

    public static Trigger shootNeutralZoneWithTrim() {
      return new Trigger(() -> false);
    }

    public static Trigger shootDashboard() {
      return leftButtonBoard.button(10, eventLoop).castTo(Trigger::new);
    }

    public static Trigger resetShooterTrims() {
      return new Trigger(() -> false);
      // return primaryController.povRight();
    }

    public static Trigger increaseDistanceTrim() {
      return leftButtonBoard
          .axisLessThan(1, -controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger decreaseDistanceTrim() {
      return leftButtonBoard
          .axisGreaterThan(1, controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger resetDistanceTrim() {
      return leftButtonBoard.button(8, eventLoop).castTo(Trigger::new);
    }

    public static Trigger trimRight() {
      return rightButtonBoard
          .axisLessThan(0, -controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger trimLeft() {
      return rightButtonBoard
          .axisGreaterThan(0, controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger trimForward() {
      return rightButtonBoard
          .axisGreaterThan(1, controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger trimBack() {
      return rightButtonBoard
          .axisLessThan(1, -controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger resetXYTrim() {
      return leftButtonBoard.button(9, eventLoop).castTo(Trigger::new);
    }

    public static final class FlywheelOI {
      public static Trigger runFlywheel() {
        return new Trigger(() -> false);
        // return primaryController.rightBumper(eventLoop);
      }

      public static Trigger increaseFlywheelTrim() {
        return new Trigger(() -> false);
      }

      public static Trigger decreaseFlywheelTrim() {
        return new Trigger(() -> false);
      }
    }

    public static final class HoodOI {
      public static Trigger hoodManualUp() {
        // Just use hood rezero
        return new Trigger(() -> false);
        // return primaryController.povUp();
      }

      public static Trigger hoodManualDown() {
        return new Trigger(() -> false);
        // return primaryController.povDown();
      }

      public static Trigger rezeroHood() {
        return leftButtonBoard.button(2, eventLoop).castTo(Trigger::new);
      }

      public static Trigger increaseHoodTrim() {
        // Just use distance trim
        return new Trigger(() -> false);
      }

      public static Trigger decreaseHoodTrim() {
        return new Trigger(() -> false);
      }
    }

    public static final class TurretOI {
      public static Trigger turretManualCCW() {
        return new Trigger(() -> false);
        // return leftButtonBoard
        //     .axisLessThan(0, -controllerStickDeadband, eventLoop)
        //     .castTo(Trigger::new);
      }

      public static Trigger turretManualCW() {
        return new Trigger(() -> false);
        // return leftButtonBoard
        //     .axisGreaterThan(0, controllerStickDeadband, eventLoop)
        //     .castTo(Trigger::new);
      }

      public static Trigger enableAutoTurret() {
        // return new Trigger(() -> false);
        return rightButtonBoard.button(9, eventLoop).castTo(Trigger::new);
      }

      public static Trigger resetTurretTrim() {
        return leftButtonBoard.button(4, eventLoop).castTo(Trigger::new);
      }

      public static Trigger increaseTurretTrim() {
        return leftButtonBoard
            .axisLessThan(0, -controllerStickDeadband, eventLoop)
            .castTo(Trigger::new);
      }

      public static Trigger decreaseTurretTrim() {
        return leftButtonBoard
            .axisGreaterThan(0, controllerStickDeadband, eventLoop)
            .castTo(Trigger::new);
      }

      public static Trigger rezeroTurret() {
        return leftButtonBoard.button(3, eventLoop).castTo(Trigger::new);
      }

      public static Trigger unmurderTurret() {
        return leftButtonBoard.button(5, eventLoop).castTo(Trigger::new);
      }

      public static Trigger lockTurret() {
        return leftButtonBoard.button(6, eventLoop).castTo(Trigger::new);
      }

      public static Trigger setTurretEncoderTo0() {
        return new Trigger(() -> false);
        // return leftButtonBoard.button(10, eventLoop).castTo(Trigger::new);
      }

      public static Trigger turretTo180() {
        return new Trigger(() -> false);
        // return leftButtonBoard.button(8, eventLoop).castTo(Trigger::new);
      }

      public static Trigger turretTo90() {
        return new Trigger(() -> false);
        // return leftButtonBoard.button(7, eventLoop).castTo(Trigger::new);
      }
    }
  }
}
