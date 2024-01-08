package frc.robot.shooter;

import frc.lib.CAN;

/** Constants for the shooter subsystem. */
public class ShooterConstants {

  /** Constants for the sensors used in the shooter subsystem. */
  public static class SensorConstants {
    /** The beam break sensor's DIO port number. */
    public static final int BEAM_BREAK_PORT = 0; // TODO
  }

  /** Constants for the serializer motor used in the shooter subsystem. */
  public static class SerializerConstants {
    /** The serializer motor controller's CAN identifier. */
    public static final CAN ID = new CAN(0); // TODO

    /** If true, invert the serializer motor controller's direction. */
    public static final boolean IS_INVERTED = false; // TODO
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** The flywheel motor controller's CAN identifier. */
    public static final CAN ID = new CAN(0); // TODO

    /** If true, invert the flywheel motor controller's direction. */
    public static final boolean IS_INVERTED = false; // TODO
  }
}
