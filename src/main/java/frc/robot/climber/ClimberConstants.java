package frc.robot.climber;

import edu.wpi.first.math.util.Units;

/** Constants for the climber subsystem. */
public class ClimberConstants {

  /** Constants for the elevator. */
  public static class ElevatorConstants {
    /** Minimum height of the elevator in meters. */
    public static final double MIN_HEIGHT = 0.0;

    /** Maximum height of the elevator in meters. */
    public static final double MAX_HEIGHT = Units.inchesToMeters(28);

    /** Voltage to be applied while going up. */
    public static final double UP_VOLTAGE = 2;

    /** Voltage to be applied wihle going down. */
    public static final double DOWN_VOLTAGE = 1;
  }
}
