package frc.robot.arm;

import edu.wpi.first.math.util.Units;
import frc.lib.CAN;

/** Constants for the arm subsystem. */
public class ArmConstants {

  /** Constants for the shoulder motor. */
  public static class ShoulderMotorConstants {
    public static final CAN CAN = new CAN(32);

    /** Gearing between the soulder motor and the shoulder joint. */
    public static final double GEARING = 51.2;

    /** Maximum voltage that can be applied to the shoulder motor. */
    public static final double MAXIMUM_VOLTAGE = 2.0;

    /** Shoulder pivot to elbow pivot distance in meters. */
    public static final double SHOULDER_TO_ELBOW_DISTANCE = Units.inchesToMeters(16.775);
  }

  /** Constants for the elbow motor. */
  public static class ElbowMotorConstants {
    /** Maximum voltage that can be applied to the elbow motor. */
    public static final double MAXIMUM_VOLTAGE = 2.0;

    /** Elbow pivot to wrist pivot distance in meters. */
    public static final double ELBOW_TO_WRIST_DISTANCE = Units.inchesToMeters(16.825);
  }
}
