package frc.robot.climber;

import edu.wpi.first.math.util.Units;

/** Constants for the climber subsystem. */
public class ClimberConstants {

  /** Constants for the elevator. */
  public static class ElevatorConstants {
    /** Gearing between the elevator motor and drum. */
    public static final double GEARING = 25.0;

    /** Mass of the elevator carriage in kilograms. */
    // TODO Does not include the mass of two WCP-0418 parts
    public static final double MASS = Units.lbsToKilograms(0.678);

    /** Diameter of the drum in meters. */
    public static final double DRUM_DIAMETER = Units.inchesToMeters(0.908);

    /** Minimum height of the elevator in meters. */
    // TODO Determine if the minimum height is above this
    public static final double MIN_HEIGHT = 0.0;

    /** Maximum height of the elevator in meters. */
    public static final double MAX_HEIGHT = Units.inchesToMeters(28);

    /** Voltage to be applied while going up. */
    public static final double UP_VOLTAGE = 2;

    /** Voltage to be applied wihle going down. */
    public static final double DOWN_VOLTAGE = 1;
  }
}
