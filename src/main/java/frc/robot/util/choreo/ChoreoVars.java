// spotless:off
package frc.robot.util.choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final LinearVelocity ClimbMaxVelocity = Units.MetersPerSecond.of(0.5);
    public static final LinearAcceleration MaxAcceleration = Units.MetersPerSecondPerSecond.of(1.75);
    public static final LinearVelocity MaxVelocity = Units.MetersPerSecond.of(2);

    public static final class Poses {
        public static final Pose2d BumpDepot = new Pose2d(4.6228, 5.7832, Rotation2d.fromRadians(0));
        public static final Pose2d BumpOutpost = new Pose2d(4.6228, 2.286, Rotation2d.fromRadians(0));
        public static final Pose2d ClimbDepot = new Pose2d(1.0534, 4.306, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d ClimbOutpost = new Pose2d(1.0534, 2.9572, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d ClimbSetupDepot = new Pose2d(1.0534, 4.9, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d ClimbSetupOutpost = new Pose2d(1.0534, 2.65, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d DepotIntake = new Pose2d(1.0858, 5.9692, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d Hub = new Pose2d(4.6204188, 4.0346312, Rotation2d.fromRadians(0));
        public static final Pose2d LeftBumpStartingPosition = new Pose2d(3.5820408, 6.0364919, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d NZDepotShootingTarget = new Pose2d(3, 6.5, Rotation2d.fromRadians(0));
        public static final Pose2d NZOutpostShootingTarget = new Pose2d(3, 1.5, Rotation2d.fromRadians(0));
        public static final Pose2d RightBumpStartingPosition = new Pose2d(3.5820408, 2.0327081, Rotation2d.fromRadians(3.1415927));
    }
}
// spotless:on
