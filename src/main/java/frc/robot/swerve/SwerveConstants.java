package frc.robot.swerve;

import edu.wpi.first.math.util.Units;
import frc.lib.config.FeedbackControllerConfig;
import frc.lib.config.FeedforwardControllerConfig;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MotorConfig;

/** Constants for the swerve subsystem. */
public class SwerveConstants {

  /** Constants for the MK4i COTS module. */
  public static class MK4iConstants {
    /** Gearing between the steer motor and the wheel. */
    public static final double STEER_GEARING = 150.0 / 7.0;

    /** Gearing between the drive motor and the wheel. */
    public static final double DRIVE_GEARING = 6.75;

    /** Diameter of the MK4i's wheels in meters. */
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

    /** Conversion between wheel rotations and distances in meters. */
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  }

  /** Drive motor config. */
  public static final MechanismConfig DRIVE_CONFIG =
      new MechanismConfig()
          .withMotorConfig(
              new MotorConfig()
                  .withCCWPositive(false)
                  .withCurrentLimit(40)
                  .withMotorToMechanismRatio(MK4iConstants.DRIVE_GEARING))
          .withFeedforwardConfig(
              new FeedforwardControllerConfig()
                  .withStaticFeedforward(0.14) // volts
                  .withVelocityFeedforward(0.725) // volts per rotation per second
              )
          .withFeedbackConfig(
              new FeedbackControllerConfig()
                  .withProportionalGain(0.75) // volts per rotation per second
              );

  /** Steer motor config. */
  public static final MechanismConfig STEER_CONFIG =
      new MechanismConfig()
          .withMotorConfig(
              new MotorConfig()
                  .withCCWPositive(false)
                  .withCurrentLimit(20)
                  .withMotorToMechanismRatio(MK4iConstants.STEER_GEARING))
          .withFeedforwardConfig(
              new FeedforwardControllerConfig().withStaticFeedforward(0.205) // volts
              )
          .withFeedbackConfig(
              new FeedbackControllerConfig()
                  .withContinuousInput(true)
                  .withProportionalGain(54.0) // volts per rotation
                  .withDerivativeGain(0.16) // volts per rotation per second
                  .withPositionTolerance(Units.degreesToRotations(1)) // rotations
              );
}
