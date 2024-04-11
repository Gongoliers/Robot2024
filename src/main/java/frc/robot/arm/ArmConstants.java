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
      PIDF.kS = 0.14;
      PIDF.kG = 0.5125;
      PIDF.kV = 4.0;
      PIDF.kP = 4.0;
    }

    /** Shoulder's controller constants. */
    public static final PositionControllerIOConstants CONTROLLER_CONSTANTS =
        new PositionControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositiveMotor = true;
      CONTROLLER_CONSTANTS.ccwPositiveAbsoluteEncoder = false;
      CONTROLLER_CONSTANTS.neutralBrake = true;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 39.771428571;
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

    public static final Rotation2d STOW = Rotation2d.fromDegrees(-26);

    public static final Rotation2d LOB = Rotation2d.fromDegrees(-26);

    public static final Rotation2d SUBWOOFER = Rotation2d.fromDegrees(-15);

    public static final Rotation2d PODIUM = Rotation2d.fromDegrees(-10);

    public static final Rotation2d EJECT = Rotation2d.fromDegrees(30);

    public static final Rotation2d SKIM = Rotation2d.fromDegrees(30);

    public static final Rotation2d AMP = Rotation2d.fromDegrees(80);
  }
}
