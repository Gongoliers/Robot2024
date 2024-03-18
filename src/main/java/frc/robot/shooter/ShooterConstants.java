package frc.robot.shooter;

/** Constants for the shooter subsystem. */
public class ShooterConstants {

  /** Constants for the serializer motor used in the shooter subsystem. */
  public static class SerializerConstants {
    /** Velocity to apply while intaking in rotations per second. */
    public static final double INTAKE_VELOCITY = 4.75;

    /** Velocity to apply while serializing in rotations per second. */
    public static final double SERIALIZE_VELOCITY = 4.75;

    /** Maximum tengential speed in meters per second. */
    public static final double MAXIMUM_TANGENTIAL_SPEED = 4.75;
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** Velocity to apply while shooting in rotations per second. */
    public static final double SHOOT_VELOCITY = 5.65;

    /** Maximum tangential speed in meters per second. */
    public static final double MAXIMUM_TANGENTIAL_SPEED = 5.65;
  }
}
