package frc.robot.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.lib.CAN;
import frc.lib.config.FeedbackControllerConfig;
import frc.lib.config.FeedforwardControllerConfig;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MotionProfileConfig;
import frc.lib.config.MotorConfig;

/** Constants for the shooter subsystem. */
public class ShooterConstants {

  /** Constants for the serializer motor used in the shooter subsystem. */
  public static class SerializerConstants {
    /** Serializer's CAN. */
    public static final CAN CAN = new CAN(42);

    /** Serializer's config. */
    public static final MechanismConfig CONFIG =
        new MechanismConfig()
            .withMotorConfig(
                new MotorConfig()
                    .withCCWPositive(true)
                    .withNeutralBrake(false)
                    .withMotorToMechanismRatio(36.0 / 16.0))
            .withMotionProfileConfig(
                new MotionProfileConfig()
                    .withMaximumVelocity(45) // rotations per second
                    .withMaximumAcceleration(450) // rotations per second per second
                )
            .withFeedforwardConfig(
                new FeedforwardControllerConfig()
                    .withStaticFeedforward(0.14) // volts
                    .withVelocityFeedforward(0.2617) // volts per rotation per second
                );

    public static final double INTAKE_SPEED = 34;

    public static final double PULL_SPEED = -10;

    public static final double EJECT_SPEED = -44;

    public static final double SLOW_FEED_SPEED = 20;

    public static final double FAST_FEED_SPEED = 44;

    public static final SlewRateLimiter ACCELERATION_LIMITER =
        CONFIG.motionProfileConfig().createRateLimiter();

    public static final double NOTE_AMPS = 20;

    public static final double TOLERANCE = 5;
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** Flywheel's CAN. */
    public static final CAN CAN = new CAN(44);

    /** Flywheel's config. */
    public static final MechanismConfig CONFIG =
        new MechanismConfig()
            .withMotorConfig(
                new MotorConfig()
                    .withCCWPositive(false)
                    .withNeutralBrake(true)
                    .withMotorToMechanismRatio(28.0 / 16.0))
            .withMotionProfileConfig(
                new MotionProfileConfig()
                    .withMaximumVelocity(60) // rotations per second
                    .withMaximumAcceleration(200) // rotations per second per second
                )
            .withFeedforwardConfig(
                new FeedforwardControllerConfig()
                    .withStaticFeedforward(0.14) // volts
                    .withVelocityFeedforward(0.2) // volts per rotation per second
                )
            .withFeedbackConfig(
                new FeedbackControllerConfig()
                    .withProportionalGain(0.14) // volts per rotation per second
                );

    public static final double PULL_SPEED = -20;

    public static final double SPEAKER_SPEED = 60;

    public static final double PODIUM_SPEED = 60;

    public static final double LOB_SPEED = 60;

    public static final double SKIM_SPEED = 60;

    public static final double AMP_SPEED = 10;

    public static final double BLOOP_SPEED = 30;

    public static final SlewRateLimiter ACCELERATION_LIMITER =
        CONFIG.motionProfileConfig().createRateLimiter();

    public static final double TOLERANCE = 5;
  }
}
