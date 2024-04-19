package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
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
      PIDF.kS = 0.14; // volts
      PIDF.kG = 0.5125; // volts
      PIDF.kV = 4.0; // volts per rotation per second
      PIDF.kP = 4.0; // volts per rotation
    }

    /** Shoulder's controller constants. */
    public static final PositionControllerIOConstants CONTROLLER_CONSTANTS =
        new PositionControllerIOConstants();

    static {
      // The leading shoulder motor has the correct reference frame
      CONTROLLER_CONSTANTS.ccwPositiveMotor = true;
      
      // The shoulder's absolute encoder has the wrong reference frame
      CONTROLLER_CONSTANTS.ccwPositiveAbsoluteEncoder = false;
      
      // Use brake mode to hold the arm in place
      // Since the arm rests on a hard stop this isn't strictly necessary,
      // but it prevents the arm from being knocked around if disabled
      CONTROLLER_CONSTANTS.neutralBrake = true;
      
      // Ask CAD if they can calculate this reduction or count the gears
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 39.771428571;

      // 1. Rest the arm in the stow position
      // 2. Use a digital level to determine the actual angle of the stow position
      // 3. Observe the value reported by the absolute encoder while the arm is in the stow position
      // 4. Calculate the absolute encoder offset by subtracting the absolute encoder value from the actual angle
      CONTROLLER_CONSTANTS.absoluteEncoderOffsetRotations = Units.degreesToRotations(-173.135);
    }

    /** Maximum speed of the shoulder in rotations per second. */
    public static final double MAXIMUM_SPEED = Units.degreesToRotations(240.0);

    /** Maximum acceleration of the shoulder in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION =
        MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.25);

    /** Maximum speed and acceleration of the shoulder . */
    public static final TrapezoidProfile.Constraints CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAXIMUM_SPEED, MAXIMUM_ACCELERATION);

    /** Motion profile of the shoulder. */
    public static final TrapezoidProfile MOTION_PROFILE = new TrapezoidProfile(CONSTRAINTS);

    /** Shoulder angle when the arm is stowed. */
    public static final Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(-26);

    /** Shoulder angle when the shooter is parallel to the ground. Used for flat shots. */
    public static final Rotation2d FLAT_ANGLE = Rotation2d.fromDegrees(30);
  }
}
