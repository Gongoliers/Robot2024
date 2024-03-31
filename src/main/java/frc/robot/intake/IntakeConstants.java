package frc.robot.intake;

/** Constants for the intake subsystem. */
public class IntakeConstants {
  /** Constants for the roller motor. */
  public static class RollerConstants {
    /** Velocity to apply when intaking in rotations per second. */
    public static final double INTAKE_VELOCITY = 1.0;

    /** Velocity to apply when outtaking in rotations per second. */
    public static final double OUTTAKE_VELOCITY = -1.0;

    /** Maximum speed of the roller in rotations per second. */
    public static final double MAXIMUM_SPEED = 1.0;

    /** Speed tolerance of the roller in rotations per second. */
    public static final double SPEED_TOLERANCE = 0.25;
  }
}
