package frc.robot.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;

/** Constants for the shooter subsystem. */
public class ShooterConstants {

  /** Constants for the serializer motor used in the shooter subsystem. */
  public static class SerializerConstants {
    /** Velocity to apply while intaking in rotations per second. */
    public static final double INTAKE_VELOCITY = 4.75;

    /** Velocity to apply while serializing in rotations per second. */
    public static final double SERIALIZE_VELOCITY = 2.5;

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 4.75;

    /** Speed tolerance in rotations per second. */
    public static final double SPEED_TOLERANCE = 0.25;
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** Velocity to apply while shooting into the speaker in rotations per second. */
    public static final double SPEAKER_VELOCITY = 5.65;

    /** Velocity to apply while shooting into the amp in rotations per second. */
    public static final double AMP_VELOCITY = 2.5;

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 5.65;

    /** Speed tolerance in rotations per second. */
    public static final double SPEED_TOLERANCE = 0.25;

    /** Acceleration limiter. */
    public static final SlewRateLimiter ACCELERATION_LIMITER = new SlewRateLimiter(1.0);
  }
}
