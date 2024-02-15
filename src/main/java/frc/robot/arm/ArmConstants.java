package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.JointConstants;
import frc.lib.MotionProfileCalculator;

/** Constants for the arm subsystem. */
public class ArmConstants {

  /** Constants for the shoulder motor. */
  public static class ShoulderMotorConstants {
    /** Shoulder motor CAN. */
    public static final CAN CAN = new CAN(32);

    /** Joint constants for the shoulder joint. */
    public static final JointConstants JOINT_CONSTANTS =
        new JointConstants(
            Units.lbsToKilograms(3.154), // massKg
            Units.inchesToMeters(16.775), // lengthMeters
            Units.inchesToMeters(8.962869), // radiusMeters
            0.07415,
            51.2,
            DCMotor.getNEO(1), // motor
            1);

    /** Minimum angle of the shoulder joint. */
    public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(12.5);

    /** Maximum angle of the shoulder joint. */
    public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(90);

    /** Tolerance of the shoulder joint. */
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(5.0);

    /** Translation of the shoulder joint relative to the origin in meters. */
    public static final Translation2d SHOULDER_TO_ORIGIN =
        new Translation2d(Units.inchesToMeters(-11.361), Units.inchesToMeters(7.721));

    /** Proportional gain in volts per rotation. */
    public static final double KP = 36.0;

    /** Maximum speed of the shoulder joint in rotations per second. */
    public static final double MAXIMUM_SPEED = 1.0;

    /** Maximum acceleration of the shoulder joint in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION =
        MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.5);

    /** Maximum speed and acceleration of the shoulder joint. */
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAXIMUM_SPEED, MAXIMUM_ACCELERATION);

    /** Motion profile of the shoulder joint using constraints. */
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(CONSTRAINTS);
  }

  /** Constants for the elbow motor. */
  public static class ElbowMotorConstants {
    /** Elbow motor CAN. */
    public static final CAN CAN = new CAN(32); // TODO

    /** Joint constants for the elbow joint. */
    public static final JointConstants JOINT_CONSTANTS =
        new JointConstants(
            Units.lbsToKilograms(13.006), // massKg
            Units.inchesToMeters(16.825), // lengthMeters
            Units.inchesToMeters(12.251799915), // radiusMeters
            0.5713,
            39.2911765,
            DCMotor.getNEO(1), // motor
            1);

    /** Minimum angle of the elbow joint. */
    public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-90);

    /** Maximum angle of the elbow joint. */
    public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(180);

    /** Tolerance of the elbow joint. */
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(5.0);

    /** Proportional gain in volts per rotation. */
    public static final double KP = 48.0;

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
}
