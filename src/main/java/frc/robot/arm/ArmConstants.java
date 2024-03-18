package frc.robot.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.JointConstants;

/** Constants for the arm subsystem. */
public class ArmConstants {

  /** Constants for the shoulder motor. */
  public static class ShoulderMotorConstants {
    /** Joint constants for the shoulder joint. */
    public static final JointConstants JOINT_CONSTANTS =
        new JointConstants(
            Units.lbsToKilograms(3.154), // massKg
            Units.inchesToMeters(16.775), // lengthMeters
            Units.inchesToMeters(8.962869), // radiusMeters
            0.07415,
            51.2,
            DCMotor.getNEO(1), // motor
            1);
  }

  /** Constants for the wrist motor. */
  public static class WristMotorConstants {
    /** Joint constants for the wrist joint. */
    public static final JointConstants JOINT_CONSTANTS =
        new JointConstants(
            Units.lbsToKilograms(8.016), // massKg
            Units.inchesToMeters(5.135), // lengthMeters
            Units.inchesToMeters(3.47629), // radiusMeters
            0.02835,
            20.454545,
            DCMotor.getNEO(1), // motor
            1);
  }
}
