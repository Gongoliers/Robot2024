package frc.robot.shooter;

import edu.wpi.first.math.util.Units;
import frc.lib.CAN;

/** Constants for the shooter subsystem. */
public class ShooterConstants {

  /** Constants for the serializer motor used in the shooter subsystem. */
  public static class SerializerConstants {
    /** Serializer motor controller's CAN identifier. */
    public static final CAN ID = new CAN(0); // TODO

    /** If true, invert the serializer motor controller's direction. */
    public static final boolean IS_INVERTED = false; // TODO

    /** Gearing between the serializer motor and the serializer wheel. */
    public static final double GEARING = 1.0; // TODO

    /** Moment of interia of the serializer wheel in joules kilograms meters squared. */
    public static final double MOI = 0.000125; // TODO

    /** Radius of the serializer wheel in meters. */
    public static final double RADIUS = 0.5 * Units.inchesToMeters(4.0);

    /** Velocity to apply while intaking in rotations per second. */
    public static final double INTAKE_VELOCITY = 4.75;

    /** Velocity to apply while serializing in rotations per second. */
    public static final double SERIALIZE_VELOCITY = 4.75;

    /** Maximum tengential speed in meters per second. */
    public static final double MAXIMUM_TANGENTIAL_SPEED = 4.75;
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** Radius of the flywheel wheel in meters. */
    public static final double RADIUS = 0.5 * Units.inchesToMeters(4.0);

    /** Velocity to apply while shooting in rotations per second. */
    public static final double SHOOT_VELOCITY = 5.65;

    /** Maximum tangential speed in meters per second. */
    public static final double MAXIMUM_TANGENTIAL_SPEED = 5.65;
  }
}
