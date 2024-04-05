package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.MotionProfileCalculator;

public class SuperstructureConstants {

  public static class ShoulderAngleConstants {
    public static final Rotation2d INITIAL = Rotation2d.fromDegrees(-26.45);

    public static final Rotation2d STOW = Rotation2d.fromDegrees(-26.45);

    public static final Rotation2d SHOOT = Rotation2d.fromDegrees(-15);

    public static final Rotation2d EJECT = Rotation2d.fromDegrees(0);

    public static final Rotation2d AMP = Rotation2d.fromDegrees(90);

    /** Tolerance of the shoulder joint. */
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);

    /** Maximum speed of the shoulder joint in rotations per second. */
    public static final double MAXIMUM_SPEED = Units.degreesToRotations(60.0);

    /** Maximum acceleration of the shoulder joint in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION =
        MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.1);

    /** Maximum speed and acceleration of the shoulder joint. */
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAXIMUM_SPEED, MAXIMUM_ACCELERATION);

    /** Motion profile of the shoulder joint using constraints. */
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(CONSTRAINTS);
  }
}
