package frc.robot.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;

/** Constants for the shooter subsystem. */
public class ShooterConstants {

  /** Constants for the serializer motor used in the shooter subsystem. */
  public static class SerializerConstants {
    /** Velocity to apply while intaking in rotations per second. */
    public static final double INTAKE_VELOCITY = 34;

    /** Velocity to apply while pulling in rotations per second. */
    public static final double PULL_VELOCITY = -5;

    /** Velocity to apply while serializing in rotations per second. */
    public static final double SERIALIZE_VELOCITY = 20;

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 45.319;

    /** Speed tolerance in rotations per second. */
    public static final double SPEED_TOLERANCE = 2.5;
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** Velocity to apply while shooting into the speaker in rotations per second. */
    public static final double SPEAKER_VELOCITY = 24;

    /** Velocity to apply while passing in rotations per second. */
    public static final double PASS_VELOCTY = 44;

    /** Velocity to apply while shooting into the amp in rotations per second. */
    public static final double AMP_VELOCITY = 20;

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 46.711;

    /** Speed tolerance in rotations per second. */
    public static final double SPEED_TOLERANCE = 1.0;

    /** Acceleration limiter. */
    public static final SlewRateLimiter ACCELERATION_LIMITER = new SlewRateLimiter(46.711);
  }
}
