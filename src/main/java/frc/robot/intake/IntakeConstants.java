package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.MotionProfileCalculator;

/** Constants for the intake subsystem. */
public class IntakeConstants {

  /** Constants for the pivot motor. */
  public static class PivotMotorConstants {
    /** Gearing between the pivot sensor and the pivot. */
    public static final double SENSOR_GEARING = 2.3;

    /** Gearing between the motor and the pivot. */
    public static final double MOTOR_GEARING = 49 * SENSOR_GEARING;

    /** Distance between the pivot and the far edge of the intake. */
    public static final double DISTANCE = Units.inchesToMeters(10.275);

    /** Pivot motor's minimum angle. */
    public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-48);

    /** Pivot motor's maximum angle. */
    public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(86);

    /** Angle between the measured intake position and the mass's position. */
    public static final Rotation2d MASS_OFFSET = Rotation2d.fromDegrees(-34.34);

    /** Pivot motor's "out" angle. */
    public static final Rotation2d OUT_ANGLE = Rotation2d.fromDegrees(0);

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

  /** Constants for the roller motor. */
  public static class RollerMotorConstants {
    /** Gearing between the roller motor and the rollers. */
    public static final double GEARING = 4.5;

    /** Velocity to apply when intaking in rotations per second. */
    public static final double INTAKE_VELOCITY = 1.0;

    /** Velocity to apply when outtaking in rotations per second. */
    public static final double OUTTAKE_VELOCITY = -1.0;

    /** Radius of the roller in meters. */
    public static final double INTAKE_ROLLER_RADIUS = 0.5 * Units.inchesToMeters(1.375);

    /** Maximum speed of the roller in rotations per second. */
    public static final double MAXIMUM_SPEED = 0.31;
  }
}
