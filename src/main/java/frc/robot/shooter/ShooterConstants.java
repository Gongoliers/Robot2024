package frc.robot.shooter;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;

/** Constants for the shooter subsystem. */
public class ShooterConstants {

  /** Constants for the sensors used in the shooter subsystem. */
  public static class SensorConstants {
    /** Beam break sensor's DIO port number. */
    public static final int BEAM_BREAK_PORT = 0; // TODO

    /** Beam break sensor's debounce filter's period in seconds. */
    public static final double BEAM_BREAK_DEBOUNCE_PERIOD = 0.1; // TODO
    /** Beam break sensor's debounce filter's type. */
    public static final DebounceType BEAM_BREAK_DEBOUNCE_TYPE = DebounceType.kBoth;
  }

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
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** Flywheel motor controller's CAN identifier. */
    public static final CAN ID = new CAN(0); // TODO

    /** If true, invert the flywheel motor controller's direction. */
    public static final boolean IS_INVERTED = false; // TODO

    /** Gearing between the flywheel motor and the flywheel wheel. */
    public static final double GEARING = 1.0; // TODO

    /** Moment of interia of the flywheel wheel in joules kilograms meters squared. */
    public static final double MOI = 0.000125; // TODO

    /** Radius of the flywheel wheel in meters. */
    public static final double RADIUS = 0.5 * Units.inchesToMeters(4.0);
  }
}
