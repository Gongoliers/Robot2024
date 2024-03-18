package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.MotionProfileCalculator;

public class SuperstructureConstants {

  public static class ShoulderAngleConstants {
    /** Minimum angle of the shoulder joint. */
    public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(29.5);

    /** Maximum angle of the shoulder joint. */
    public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);

    /** Tolerance of the shoulder joint. */
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(5.0);

    /** Maximum speed of the shoulder joint in rotations per second. */
    public static final double MAXIMUM_SPEED = 0.5;

    /** Maximum acceleration of the shoulder joint in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION =
        MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.1);

    /** Maximum speed and acceleration of the shoulder joint. */
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAXIMUM_SPEED, MAXIMUM_ACCELERATION);

    /** Motion profile of the shoulder joint using constraints. */
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(CONSTRAINTS);
  }

  public static class WristAngleConstants {
    /** Minimum angle of the wrist joint. */
    public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-86.759);

    /** Maximum angle of the wrist joint. */
    public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(85.98);

    /** Tolerance of the wrist joint. */
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(5.0);

    /** Maximum speed of the wrist joint in rotations per second. */
    public static final double MAXIMUM_SPEED = 1.2;

    /** Maximum acceleration of the wrist joint in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION =
        MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.25);

    /** Maximum speed and acceleration of the wrist joint. */
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAXIMUM_SPEED, MAXIMUM_ACCELERATION);

    /** Motion profile of the wrist joint using constraints. */
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(CONSTRAINTS);
  }

  public static class IntakePivotAngleConstants {
    /** Pivot motor's minimum angle. */
    public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-48);

    /** Pivot motor's maximum angle. */
    public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(86);

    /** Pivot motor's tolerance. */
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(8.0);

    /** Maximum speed of the pivot in rotations per second. */
    public static final double MAXIMUM_SPEED = 1;

    /** Maximum acceleration of the pivot in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION =
        MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.1);

    /** Maximum speed and acceleration of the pivot. */
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAXIMUM_SPEED, MAXIMUM_ACCELERATION);

    /** Motion profile of the pivot using constraints. */
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(CONSTRAINTS);
  }
}
