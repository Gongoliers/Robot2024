package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.MotionProfileCalculator;
import frc.lib.MotorCurrentLimits;

/** Constants for the intake subsystem. */
public class IntakeConstants {

  /** Constants for the pivot motor. */
  public static class PivotMotorConstants {
    /** Pivot motor's CAN identifier. */
    public static final CAN CAN = new CAN(8);

    /** If true, invert the pivot motor. */
    public static final boolean IS_MOTOR_INVERTED = false;

    /** If true, invert the pivot motor sensor. */
    public static final boolean IS_SENSOR_INVERTED = true;

    /** Gearing between the pivot sensor and the pivot. */
    public static final double SENSOR_GEARING = 2.3;

    /** Gearing between the motor and the pivot. */
    public static final double MOTOR_GEARING = 49 * SENSOR_GEARING;

    /** Pivot motor's moment of interia in kilograms meters squared. */
    public static final double MOI = 0.02; // TODO

    /** Distance between the pivot and the far edge of the intake. */
    public static final double DISTANCE = Units.inchesToMeters(10.275);

    /** Pivot motor's minimum angle. */
    public static final Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-60);

    /** Pivot motor's maximum angle. */
    public static final Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(86);

    /** Pivot motor's "out" angle. */
    public static final Rotation2d OUT_ANGLE = Rotation2d.fromDegrees(-10);

    /** Pivot motor's tolerance. */
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(4.0);

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

    /** Proportional gain of the arm in volts per rotation. */
    public static final double KP = 16.0;
  }

  /** Constants for the roller motor. */
  public static class RollerMotorConstants {
    /** Roller motor's CAN identifier. */
    public static final CAN CAN = new CAN(5);

    /** If true, invert the roller motor. */
    public static final boolean IS_INVERTED = false;

    /** Gearing between the roller motor and the rollers. */
    public static final double GEARING = 4.5;

    /** Moment of inertia of the roller in joules kilograms meters squared. */
    public static final double MOI = 0.5343;

    /** Voltage to apply when intaking in volts. */
    // 12 volts produces enough speed to unscrew the dead axle
    public static final double INTAKE_VOLTAGE = 11;

    /** Voltage to apply when outtaking in volts. */
    public static final double OUTTAKE_VOLTAGE = -8;

    /** Current limits for the roller motor. */
    public static final MotorCurrentLimits CURRENT_LIMITS = new MotorCurrentLimits(40);

    /** Radius of the roller in meters. */
    public static final double INTAKE_ROLLER_RADIUS = 0.5 * Units.inchesToMeters(1.375);

    /** Size of the current spike when intaking a note in amps. */
    public static final double NOTE_CURRENT = 18.0; // TODO

    /** Duration of the current spike when intaking a note. */
    public static final double STALL_DURATION = 0.35;
  }

  /** Constants for intake commands. */
  public static class IntakeCommandConstants {
    /** Delay between starting intaking and detecting notes in seconds. */
    public static final double NOTE_DETECTION_DELAY = 0.5;
  }
}
