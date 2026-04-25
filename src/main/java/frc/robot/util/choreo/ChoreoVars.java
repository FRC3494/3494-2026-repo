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
    public static final LinearVelocity DepotMaxVelocity = Units.MetersPerSecond.of(0.6);
    public static final Distance FieldLength = Units.Meters.of(16.541);
    public static final Distance FieldWidth = Units.Meters.of(8.0692);
    public static final LinearVelocity IntakeMaxVelocity = Units.MetersPerSecond.of(1.25);
    public static final LinearAcceleration MaxAcceleration = Units.MetersPerSecondPerSecond.of(2);
    public static final LinearVelocity MaxVelocity = Units.MetersPerSecond.of(2);
    public static final LinearAcceleration NZMaxAcceleration = Units.MetersPerSecondPerSecond.of(4);
    public static final LinearVelocity NZMaxVelocity = Units.MetersPerSecond.of(4.53);
    public static final Distance RobotBumperSize = Units.Meters.of(0.447675);
    public static final LinearVelocity RobotMaxSpeed = Units.MetersPerSecond.of(4.63);
    public static final LinearVelocity ShootingMaxSpeed = Units.MetersPerSecond.of(1);

    public static final class Poses {
        public static final Pose2d BumpDepot = new Pose2d(4.6228, 5.7832, Rotation2d.fromRadians(0));
        public static final Pose2d BumpOutpost = new Pose2d(4.6228, 2.286, Rotation2d.fromRadians(0));
        public static final Pose2d ClimbDepot = new Pose2d(1.1042, 4.5038, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d ClimbOutpost = new Pose2d(1.0788, 2.9572, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d ClimbSetupDepot = new Pose2d(1.1042, 4.9, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d ClimbSetupOutpost = new Pose2d(1.0788, 2.65, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d DepotIntake = new Pose2d(1.0858, 5.9692, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d DepotSideLeft = new Pose2d(0.4650392, 7.4, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d DepotSideRight = new Pose2d(0.4650392, 4.9616, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d Hub = new Pose2d(4.62534, 4.0346, Rotation2d.fromRadians(0));
        public static final Pose2d LeftBumpStartingPosition = new Pose2d(3.5820408, 6.0364919, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d LeftFarMiddleNZ = new Pose2d(8.4096, 4.92736, Rotation2d.fromRadians(-1.8325957));
        public static final Pose2d LeftFarNZ = new Pose2d(8.4096, 7.36576, Rotation2d.fromRadians(-1.8325957));
        public static final Pose2d LeftHubStartingPosition = new Pose2d(3.5788491, 4.1838441, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d LeftMiddleNZ = new Pose2d(7.8005117, 4.92736, Rotation2d.fromRadians(-1.8325957));
        public static final Pose2d LeftNZ = new Pose2d(7.8005117, 7.36576, Rotation2d.fromRadians(-1.8325957));
        public static final Pose2d LeftNZLoopEnd = new Pose2d(5.801136, 5.8199215, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d LeftTrench = new Pose2d(4.62534, 7.429882, Rotation2d.fromRadians(0));
        public static final Pose2d LeftTrenchStartingPosition = new Pose2d(4.476115, 7.443725, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d NZDepotShootingTarget = new Pose2d(1, 6.0428, Rotation2d.fromRadians(0));
        public static final Pose2d NZOutpostShootingTarget = new Pose2d(1, 1.9572, Rotation2d.fromRadians(0));
        public static final Pose2d Outpost = new Pose2d(0.4433372, 0.773997, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d RightBumpStartingPosition = new Pose2d(3.5820408, 2.0327081, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d RightMiddleNZ = new Pose2d(7.7751117, 3.26376, Rotation2d.fromRadians(1.8325957));
        public static final Pose2d RightNZ = new Pose2d(7.7751117, 0.82536, Rotation2d.fromRadians(1.8325957));
        public static final Pose2d RightNZLoopEnd = new Pose2d(5.801136, 2.2492785, Rotation2d.fromRadians(-1.5707963));
        public static final Pose2d RightTrench = new Pose2d(4.62534, 0.639318, Rotation2d.fromRadians(0));
        public static final Pose2d RightTrenchStartingPosition = new Pose2d(4.476115, 0.625475, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d ShootLeftTrench = new Pose2d(2.9277268, 7.1208358, Rotation2d.fromRadians(0.5235988));
        public static final Pose2d ShootRightTrench = new Pose2d(3.0801268, 0.690118, Rotation2d.fromRadians(0));
    }
}
// spotless:on
