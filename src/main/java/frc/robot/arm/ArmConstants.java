package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.ControllerConstants;
import frc.lib.FeedbackControllerConstants;
import frc.lib.FeedforwardControllerConstants;
import frc.lib.MotionProfileCalculator;
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
    public static final ControllerConstants PIDF_CONTROLLER_CONSTANTS =
        new ControllerConstants()
            .withFeedforward(
                new FeedforwardControllerConstants()
                    .withStaticFeedforward(0.14) // volts
                    .withGravityFeedforward(0.5125) // volts
                    .withVelocityFeedforward(4.0) // volts per rotation per second
                )
            .withFeedback(
                new FeedbackControllerConstants().withProportionalGain(4.0) // volts per rotation
                );

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

    /** Shoudler angle when stowed. */
    public static final Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(-26);

    /** Shoulder angle when shooter is parallel to the ground. */
    public static final Rotation2d FLAT_ANGLE = Rotation2d.fromDegrees(30);

    /** Shoulder angle when amping. */
    public static final Rotation2d AMP_ANGLE = Rotation2d.fromDegrees(60);
  }
}
