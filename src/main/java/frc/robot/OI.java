package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.Collections;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public final class OI implements Sendable {
  private static EventLoop eventLoop = new EventLoop();
  private static CommandXboxController primaryController =
      new CommandXboxController(primaryControllerPort);
  private static Joystick leftButtonBoard = new Joystick(leftButtonBoardPort);
  private static Joystick rightButtonBoard = new Joystick(rightButtonBoardPort);

  private static WonAutoState wonAutoState = WonAutoState.Unknown;

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addIntegerArrayProperty(
        "Rumble End Times",
        () -> ((long[]) shiftRumbleTimesSeconds),
        (long[] value) ->
            shiftRumbleTimesSeconds =
                Arrays.stream(value)
                    .boxed()
                    .sorted(Collections.reverseOrder())
                    .mapToLong(Long::longValue)
                    .toArray());

    builder.addBooleanProperty(
        "Rumble Enabled", () -> shiftRumbleEnabled, (boolean value) -> shiftRumbleEnabled = value);
    builder.addDoubleProperty(
        "Rumble Intensity",
        () -> shiftRumbleIntensity,
        (double value) -> shiftRumbleIntensity = value);
    builder.addDoubleProperty(
        "Rumble Offset",
        () -> shiftRumbleOffsetSeconds,
        (double value) -> shiftRumbleOffsetSeconds = value);
    builder.addDoubleProperty(
        "Rumble Continuous Time",
        () -> shiftRumbleContinuousSeconds,
        (double value) -> shiftRumbleContinuousSeconds = value);
    builder.addDoubleProperty(
        "Rumble Pulse On Time",
        () -> shiftRumblePulseOnSeconds,
        (double value) -> shiftRumblePulseOnSeconds = value);
    builder.addDoubleProperty(
        "Rumble Pulse Off Time",
        () -> shiftRumblePulseOffSeconds,
        (double value) -> shiftRumblePulseOffSeconds = value);
    builder.addIntegerProperty(
        "Rumble Pulse Count",
        () -> shiftRumblePulseCount,
        (long value) -> shiftRumblePulseCount = ((int) value));
  }

  public static void update() {
    eventLoop.poll();

    Logger.recordOutput("OI/JoystickDriveX", DriveOI.joystickDriveX());
    Logger.recordOutput("OI/JoystickDriveY", DriveOI.joystickDriveY());
    Logger.recordOutput("OI/JoystickDriveOmega", DriveOI.joystickDriveOmega());

    if (wonAutoState == WonAutoState.Unknown) {
      String gameData = DriverStation.getGameSpecificMessage();
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (gameData.length() > 0 && alliance.isPresent()) {
        switch (gameData.charAt(0)) {
          case 'B':
            setWonAutoState(alliance.get() == Alliance.Blue ? WonAutoState.Won : WonAutoState.Lost);
            break;
          case 'R':
            setWonAutoState(alliance.get() == Alliance.Blue ? WonAutoState.Lost : WonAutoState.Won);
            break;
          default:
            break;
        }
      }
    }
  }

  public static void setWonAutoState(WonAutoState state) {
    Logger.recordOutput("OI/WonAutoState", state.toString());
    wonAutoState = state;

    Color indicatorColor = Color.kBlack;
    switch (state) {
      case Won:
        indicatorColor = Color.kCyan;
        break;
      case Lost:
        indicatorColor = Color.kMagenta;
        break;
      case Unknown:
        break;
    }
    Logger.recordOutput("OI/WonAutoIndicator", indicatorColor.toHexString());
  }

  public static enum WonAutoState {
    Won,
    Lost,
    Unknown
  }

  // #region WHOLE ROBOT
  // #endregion

  // #region CLIMBER
  public static final class ClimberOI {
    public static Trigger climberUp() {
      return rightButtonBoard.button(6, eventLoop).castTo(Trigger::new);
      // return primaryController.y(eventLoop).or(rightButtonBoard.button(6, eventLoop));
    }

    public static Trigger climberDown() {
      return rightButtonBoard.button(5, eventLoop).castTo(Trigger::new);
    }

    public static Trigger actuallyClimb() {
      return rightButtonBoard.button(7, eventLoop).castTo(Trigger::new);
    }

    public static Trigger rezeroClimber() {
      return leftButtonBoard.button(1, eventLoop).castTo(Trigger::new);
    }

    public static Trigger climberManualUp() {
      // Right button board joystick UP
      return rightButtonBoard
          .axisGreaterThan(1, buttonBoardStickDeadband, eventLoop)
          .castTo(Trigger::new);
      // return new Trigger(() -> false);
    }

    public static Trigger climberManualDown() {
      // Right button board joystick DOWN
      return rightButtonBoard
          .axisLessThan(1, -buttonBoardStickDeadband, eventLoop)
          .castTo(Trigger::new);
      // return new Trigger(() -> false);
    }
  }
  // #endregion

  // #region DRIVE
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
      return primaryController.povLeft();
    }

    public static Trigger autoAlignClimb() {
      return primaryController.rightBumper(eventLoop);
    }

    public static Trigger autoDriveThroughTrench() {
      return primaryController.leftBumper();
    }
  }
  // #endregion

  // #region HOPPER
  public static final class HopperOI {
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
      // return primaryController.povLeft();
      return new Trigger(() -> false);
    }
  }
  // #endregion

  // #region INTAKE
  public static final class IntakeOI {
    public static Trigger intake() {
      return primaryController
          .rightTrigger(controllerTriggerDeadband, eventLoop)
          .or(rightButtonBoard.button(10, eventLoop));
    }

    public static Trigger outtake() {
      return primaryController.x(eventLoop).or(rightButtonBoard.button(3, eventLoop));
    }

    public static Trigger toggleIntake() {
      return new Trigger(() -> false);
      // return rightButtonBoard.button(10, eventLoop).castTo(Trigger::new);
    }

    public static Trigger jostleIntake() {
      return primaryController.b(eventLoop);
    }

    public static Trigger raiseIntake() {
      // Right button board joystick LEFT
      return rightButtonBoard
          .axisGreaterThan(0, controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger lowerIntake() {
      // Right button board joystick RIGHT
      return rightButtonBoard
          .axisLessThan(0, -controllerStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }
  }
  // #endregion

  // #region SHOOTER
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
      return new Trigger(() -> false);
      // return rightButtonBoard.button(3, eventLoop).castTo(Trigger::new);
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

    public static Trigger increaseDistanceTrim() {
      // Left button board joystick UP
      return leftButtonBoard
          .axisLessThan(1, -buttonBoardStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger decreaseDistanceTrim() {
      // Left button board joystick DOWN
      return leftButtonBoard
          .axisGreaterThan(1, buttonBoardStickDeadband, eventLoop)
          .castTo(Trigger::new);
    }

    public static Trigger resetDistanceTrim() {
      return leftButtonBoard.button(8, eventLoop).castTo(Trigger::new);
    }

    public static Trigger trimRight() {
      return new Trigger(() -> false);
      // // Right button board joystick RIGHT
      // return rightButtonBoard
      //     .axisLessThan(0, -buttonBoardStickDeadband, eventLoop)
      //     .castTo(Trigger::new);
    }

    public static Trigger trimLeft() {
      return new Trigger(() -> false);
      // // Right button board joystick LEFT
      // return rightButtonBoard
      //     .axisGreaterThan(0, buttonBoardStickDeadband, eventLoop)
      //     .castTo(Trigger::new);
    }

    public static Trigger trimForward() {
      return new Trigger(() -> false);
      // // Right button board joystick UP
      // return rightButtonBoard
      //     .axisGreaterThan(1, buttonBoardStickDeadband, eventLoop)
      //     .castTo(Trigger::new);
    }

    public static Trigger trimBack() {
      return new Trigger(() -> false);
      // // Right button board joystick DOWN
      // return rightButtonBoard
      //     .axisLessThan(1, -buttonBoardStickDeadband, eventLoop)
      //     .castTo(Trigger::new);
    }

    // TODO: assign this button
    public static Trigger resetXYTrim() {
      return new Trigger(() -> false);
      // return leftButtonBoard.button(9, eventLoop).castTo(Trigger::new);
    }

    // #region FLYWHEEL
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
    // #endregion

    // #region HOOD
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
    // #endregion

    // #region TURRET
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
        // Left button board joystick LEFT
        return leftButtonBoard
            .axisLessThan(0, -buttonBoardStickDeadband, eventLoop)
            .castTo(Trigger::new);
      }

      public static Trigger decreaseTurretTrim() {
        // Left button board joystick RIGHT
        return leftButtonBoard
            .axisGreaterThan(0, buttonBoardStickDeadband, eventLoop)
            .castTo(Trigger::new);
      }

      public static Trigger lockTurret() {
        return leftButtonBoard.button(6, eventLoop).castTo(Trigger::new);
      }

      public static Trigger rezeroTurret() {
        return new Trigger(() -> false);
      }
    }
    // #endregion
  }

  // #region RUMBLE
  public static final class RumbleOI {
    private static Command rumbleOn() {
      return Commands.runOnce(
              () -> {
                primaryController.setRumble(RumbleType.kBothRumble, shiftRumbleIntensity);
                Logger.recordOutput("OI/ControllerRumble", true);
              })
          .ignoringDisable(true)
          .withName("ControllerRumbleOff");
    }

    private static Command rumbleOff() {
      return Commands.runOnce(
              () -> {
                primaryController.setRumble(RumbleType.kBothRumble, 0.0);
                Logger.recordOutput("OI/ControllerRumble", false);
              })
          .ignoringDisable(true)
          .withName("ControllerRumbleOn");
    }

    public static Command shiftRumbleSequence() {
      Command pulses =
          defer(
                  () ->
                      sequence(
                              rumbleOn(),
                              Commands.waitSeconds(shiftRumblePulseOnSeconds),
                              rumbleOff(),
                              Commands.waitSeconds(shiftRumblePulseOffSeconds))
                          .repeatedly()
                          .withTimeout(
                              shiftRumblePulseCount
                                  * (shiftRumblePulseOnSeconds + shiftRumblePulseOffSeconds)),
                  Set.of())
              .withName("ControllerRumblePulses");

      Command continuous =
          defer(
                  () ->
                      sequence(rumbleOn(), waitSeconds(shiftRumbleContinuousSeconds), rumbleOff()),
                  Set.of())
              .withName("ControllerRumbleContinuous");

      return Commands.sequence(pulses, continuous)
          .finallyDo(
              () -> {
                primaryController.setRumble(RumbleType.kBothRumble, 0.0);
                Logger.recordOutput("OI/ControllerRumble", false);
              })
          .ignoringDisable(true)
          .withName("ControllerRumble");
    }

    /** True while we are inside the rumble window leading up to any shift. */
    public static Trigger shiftRumbleWindow() {
      return RobotModeTriggers.teleop()
          .and(
              () -> {
                double matchTime = DriverStation.getMatchTime();
                if (matchTime <= 0) return false;

                if (!shiftRumbleEnabled) return false;

                for (double shiftTime : shiftRumbleTimesSeconds) {
                  if (matchTime
                      > shiftTime
                          + shiftRumbleOffsetSeconds
                          + shiftRumbleContinuousSeconds
                          + shiftRumblePulseCount
                              * (shiftRumblePulseOnSeconds + shiftRumblePulseOffSeconds)) {
                    // Rumble for shiftTime has not yet started
                    // shiftTimesSeconds is descending, so no need to check other cases
                    return false;
                  }

                  if (matchTime >= shiftTime) return true;
                }

                return false;
              });
    }
  }
  // #endregion
}
