package frc.robot.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.MotionProfileCalculator;
import frc.lib.PIDFConstants;
import frc.lib.controller.PositionControllerIO.PositionControllerIOConstants;

/** Constants for the arm subsystem. */
public class ArmConstants {
  /** Constants for the shoulder. */
  public static class ShoulderConstants {
    /** Shoulder's leader CAN. */
    public static final CAN LEADER_CAN = new CAN(48);

    /** Shoulder's follower CAN. */
    public static final CAN FOLLOWER_CAN = new CAN(46);

    /** Shoulder's encoder CAN. */
    public static final CAN ENCODER_CAN = new CAN(52);

    /** Shoulder's PIDF constants. */
    public static final PIDFConstants PIDF = new PIDFConstants();

    static {
      PIDF.kS = 0.14;
      PIDF.kG = 0.45;
      PIDF.kV = 4.0;
      PIDF.kP = 20.0;
    }

    /** Shoulder's controller constants. */
    public static final PositionControllerIOConstants CONTROLLER_CONSTANTS =
        new PositionControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = true;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 39.771428571;
      CONTROLLER_CONSTANTS.absoluteEncoderOffsetRotations =
          Units.degreesToRotations(-146.77) + Units.degreesToRotations(-27.07);
    }

    /** Maximum speed of the shoulder in rotations per second. */
    public static final double MAXIMUM_SPEED = Units.degreesToRotations(60.0);

    /** Maximum acceleration of the shoulder in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION =
        MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.1);

    /** Maximum speed and acceleration of the shoulder . */
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAXIMUM_SPEED, MAXIMUM_ACCELERATION);

    /** Motion profile of the shoulder. */
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(CONSTRAINTS);
  }
}
