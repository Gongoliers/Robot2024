package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.MotionProfileCalculator;

/** Constants for the arm subsystem. */
public class ArmConstants {

  /** Constants for the shoulder motor. */
  public static class ShoulderMotorConstants {
    public static final CAN CAN = new CAN(32);

    /** Gearing between the soulder motor and the shoulder joint. */
    public static final double GEARING = 51.2;

    /** Moment of intertia of the shoulder, in kilograms meters squared. */
    public static final double MOI = 0.15093;

    /** Minumum angle of the shoulder joint. */
    public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(20);

    /** Maximum angle of the shoulder joint. */
    public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);

    /** Shoulder pivot to elbow pivot distance in meters. */
    public static final double SHOULDER_TO_ELBOW_DISTANCE = Units.inchesToMeters(16.775);

    /** Proportional gain in volts per rotation. */
    public static final double KP = 36.0;

    /** Maximum speed of the shoulder joint in rotations per second. */
    public static final double MAXIMUM_SPEED = 1.5;

    /** Maximum acceleration of the shoulder joint in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION =
        MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.3);

    /** Maximum speed and acceleration of the shoulder joint. */
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAXIMUM_SPEED, MAXIMUM_ACCELERATION);

    /** Motion profile of the shoulder joint using constraints. */
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(CONSTRAINTS);
  }

  /** Constants for the elbow motor. */
  public static class ElbowMotorConstants {
    /** Maximum voltage that can be applied to the elbow motor. */
    public static final double MAXIMUM_VOLTAGE = 2.0;

    /** Elbow pivot to wrist pivot distance in meters. */
    public static final double ELBOW_TO_WRIST_DISTANCE = Units.inchesToMeters(16.825);
  }
}
