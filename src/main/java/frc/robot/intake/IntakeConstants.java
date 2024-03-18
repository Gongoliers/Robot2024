package frc.robot.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Constants for the intake subsystem. */
public class IntakeConstants {

  /** Constants for the pivot motor. */
  public static class PivotMotorConstants {
    /** Distance between the pivot and the far edge of the intake. */
    public static final double DISTANCE = Units.inchesToMeters(10.275);

    /** Angle between the measured intake position and the mass's position. */
    public static final Rotation2d MASS_OFFSET = Rotation2d.fromDegrees(-34.34);

    /** Pivot motor's "out" angle. */
    public static final Rotation2d OUT_ANGLE = Rotation2d.fromDegrees(0);
  }

  /** Constants for the roller motor. */
  public static class RollerMotorConstants {
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
