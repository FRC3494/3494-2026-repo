package frc.robot.util.choreo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo. DO NOT MODIFY THIS FILE YOURSELF; instead,
 * change these values in the Choreo GUI.
 */
public final class ChoreoVars {
  public static final LinearVelocity ClimbMaxVelocity = Units.MetersPerSecond.of(0.5);
  public static final LinearAcceleration MaxAcceleration = Units.MetersPerSecondPerSecond.of(1.75);
  public static final LinearVelocity MaxVelocity = Units.MetersPerSecond.of(2);

  public static final class Poses {
    public static final Pose2d BumpDepot = new Pose2d(4.623, 5.783, Rotation2d.kZero);
    public static final Pose2d BumpOutpost = new Pose2d(4.623, 2.286, Rotation2d.kZero);
    public static final Pose2d ClimbDepot = new Pose2d(1.13, 4.357, Rotation2d.fromRadians(1.571));
    public static final Pose2d ClimbOutpost =
        new Pose2d(1.109, 2.957, Rotation2d.fromRadians(-1.571));
    public static final Pose2d ClimbSetupDepot =
        new Pose2d(1.13, 4.9, Rotation2d.fromRadians(1.571));
    public static final Pose2d ClimbSetupOutpost =
        new Pose2d(1.109, 2.65, Rotation2d.fromRadians(-1.571));
    public static final Pose2d DepotIntake =
        new Pose2d(1.086, 5.969, Rotation2d.fromRadians(3.142));
    public static final Pose2d Hub = new Pose2d(4.62, 4.035, Rotation2d.kZero);
    public static final Pose2d LeftBumpStartingPosition =
        new Pose2d(3.582, 6.036, Rotation2d.fromRadians(3.142));
    public static final Pose2d NZDepotShootingTarget = new Pose2d(1, 6.5, Rotation2d.kZero);
    public static final Pose2d NZOutpostShootingTarget = new Pose2d(1, 1.5, Rotation2d.kZero);
    public static final Pose2d RightBumpStartingPosition =
        new Pose2d(3.582, 2.033, Rotation2d.fromRadians(3.142));

    private Poses() {}
  }

  private ChoreoVars() {}
}
