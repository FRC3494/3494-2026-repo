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
    public static final Distance FieldLength = Units.Meters.of(16.541);
    public static final Distance FieldWidth = Units.Meters.of(8.069);
    public static final LinearVelocity IntakeMaxVelocity = Units.MetersPerSecond.of(0.75);
    public static final LinearAcceleration MaxAcceleration = Units.MetersPerSecondPerSecond.of(2);
    public static final LinearVelocity MaxVelocity = Units.MetersPerSecond.of(2);
    public static final LinearAcceleration NZMaxAcceleration = Units.MetersPerSecondPerSecond.of(4);
    public static final LinearVelocity NZMaxVelocity = Units.MetersPerSecond.of(4.53);
    public static final Distance RobotBumperSize = Units.Meters.of(0.448);
    public static final LinearVelocity RobotMaxSpeed = Units.MetersPerSecond.of(4.63);
    public static final LinearVelocity TrenchDropIntakeSpeed = Units.MetersPerSecond.of(0.5);

    public static final class Poses {
        public static final Pose2d BumpDepot = new Pose2d(4.623, 5.783, Rotation2d.kZero);
        public static final Pose2d BumpOutpost = new Pose2d(4.623, 2.286, Rotation2d.kZero);
        public static final Pose2d ClimbDepot = new Pose2d(1.053, 4.484, Rotation2d.fromRadians(1.571));
        public static final Pose2d ClimbOutpost = new Pose2d(1.053, 2.957, Rotation2d.fromRadians(-1.571));
        public static final Pose2d ClimbSetupDepot = new Pose2d(1.053, 4.9, Rotation2d.fromRadians(1.571));
        public static final Pose2d ClimbSetupOutpost = new Pose2d(1.053, 2.65, Rotation2d.fromRadians(-1.571));
        public static final Pose2d DepotIntake = new Pose2d(1.086, 5.969, Rotation2d.fromRadians(3.142));
        public static final Pose2d Hub = new Pose2d(4.625, 4.035, Rotation2d.kZero);
        public static final Pose2d LeftBumpStartingPosition = new Pose2d(3.582, 6.036, Rotation2d.fromRadians(3.142));
        public static final Pose2d LeftMiddleNZ = new Pose2d(7.775, 5.598, Rotation2d.fromRadians(-1.833));
        public static final Pose2d LeftNZ = new Pose2d(7.775, 7.274, Rotation2d.fromRadians(-1.833));
        public static final Pose2d LeftNZLoopEnd = new Pose2d(5.801, 5.82, Rotation2d.fromRadians(1.571));
        public static final Pose2d LeftTrench = new Pose2d(4.625, 7.43, Rotation2d.kZero);
        public static final Pose2d LeftTrenchStartingPosition = new Pose2d(4.476, 7.596, Rotation2d.fromRadians(3.142));
        public static final Pose2d NZDepotShootingTarget = new Pose2d(1, 6.043, Rotation2d.kZero);
        public static final Pose2d NZOutpostShootingTarget = new Pose2d(1, 1.957, Rotation2d.kZero);
        public static final Pose2d RightBumpStartingPosition = new Pose2d(3.582, 2.033, Rotation2d.fromRadians(3.142));
        public static final Pose2d RightMiddleNZ = new Pose2d(7.775, 2.471, Rotation2d.fromRadians(1.833));
        public static final Pose2d RightNZ = new Pose2d(7.775, 0.795, Rotation2d.fromRadians(1.833));
        public static final Pose2d RightNZLoopEnd = new Pose2d(5.801, 2.249, Rotation2d.fromRadians(-1.571));
        public static final Pose2d RightTrench = new Pose2d(4.625, 0.639, Rotation2d.kZero);
        public static final Pose2d RightTrenchStartingPosition = new Pose2d(4.476, 0.473, Rotation2d.fromRadians(3.142));
        public static final Pose2d ShootLeftTrench = new Pose2d(3.08, 7.502, Rotation2d.kZero);
        public static final Pose2d ShootRightTrench = new Pose2d(3.08, 0.567, Rotation2d.kZero);

        private Poses() {}
    }

    private ChoreoVars() {}
}