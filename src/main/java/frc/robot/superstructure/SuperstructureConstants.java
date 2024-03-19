package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.MotionProfileCalculator;

public class SuperstructureConstants {

  public static class ShoulderAngleConstants {
    public static final Rotation2d INITIAL = Rotation2d.fromDegrees(52.5);

    public static final Rotation2d STOW = Rotation2d.fromDegrees(29.5);

    public static final Rotation2d AMP = Rotation2d.fromDegrees(90);

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
    public static final Rotation2d INITIAL = Rotation2d.fromDegrees(-35);

    public static final Rotation2d STOW = Rotation2d.fromDegrees(85.98);

    public static final Rotation2d INTAKE = Rotation2d.fromDegrees(4);

    public static final Rotation2d SHOOT = Rotation2d.fromDegrees(18);

    public static final Rotation2d AMP = Rotation2d.fromDegrees(0);

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

  public static class PivotAngleConstants {
    public static final Rotation2d UP = Rotation2d.fromDegrees(86);

    public static final Rotation2d DOWN = Rotation2d.fromDegrees(-48);

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
