package frc.robot.arm;

import frc.lib.CAN;

/** Constants for the arm subsystem. */
public class ArmConstants {

  /** Constants for the shoulder motor. */
  public static class ShoulderMotorConstants {
    public static final CAN CAN = new CAN(32);

    /** Gearing between the soulder motor and the shoulder joint. */
    public static final double GEARING = 51.2;

    /** If true, invert the shoulder motor. */
    public static final boolean MOTOR_IS_INVERTED = true;

    /** If true, invert the shoulder motor's encoder. */
    public static final boolean ENCODER_IS_INVERTED = true;

    /** Maximum voltage that can be applied to the shoulder motor. */
    public static final double MAXIMUM_VOLTAGE = 2.0;
  }

  /** Constants for the elbow motor. */
  public static class ElbowMotorConstants {
    /** Maximum voltage that can be applied to the elbow motor. */
    public static final double MAXIMUM_VOLTAGE = 2.0;
  }
}
