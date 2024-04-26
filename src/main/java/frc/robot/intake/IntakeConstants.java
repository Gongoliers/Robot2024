package frc.robot.intake;

import frc.lib.CAN;
import frc.lib.config.FeedbackControllerConfig;
import frc.lib.config.FeedforwardControllerConfig;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MotionProfileConfig;
import frc.lib.config.MotorConfig;

/** Constants for the intake subsystem. */
public class IntakeConstants {
  /** Constants for the front roller. */
  public static class FrontRollerConstants {
    /** Front roller's CAN. */
    public static final CAN CAN = new CAN(50);

    /** Front roller's config. */
    public static final MechanismConfig CONFIG =
        new MechanismConfig()
            .withMotorConfig(
                new MotorConfig()
                    .withCCWPositive(false)
                    .withNeutralBrake(false)
                    .withMotorToMechanismRatio(24.0 / 16.0))
            .withMotionProfileConfig(
                new MotionProfileConfig().withMaximumVelocity(66) // rotations per second
                )
            .withFeedforwardConfig(
                new FeedforwardControllerConfig()
                    .withStaticFeedforward(0.13) // volts
                    .withVelocityFeedforward(0.1683) // volts per rotation per second
                )
            .withFeedbackConfig(
                new FeedbackControllerConfig()
                    .withProportionalGain(0.1) // volts per rotation per second
                );

    public static final double NOTE_AMPS = 40;
  }

  /** Constants for the back roller. */
  public static class BackRollerConstants {
    /** Back roller's CAN. */
    public static final CAN CAN = new CAN(40);

    public static final MechanismConfig CONFIG =
        new MechanismConfig()
            .withMotorConfig(
                new MotorConfig()
                    .withCCWPositive(false)
                    .withNeutralBrake(false)
                    .withMotorToMechanismRatio(24.0 / 16.0))
            .withMotionProfileConfig(
                new MotionProfileConfig().withMaximumVelocity(66) // rotations per second
                )
            .withFeedforwardConfig(
                new FeedforwardControllerConfig()
                    .withStaticFeedforward(0.13) // volts
                    .withVelocityFeedforward(0.1759) // volts per rotation per second
                )
            .withFeedbackConfig(
                new FeedbackControllerConfig()
                    .withProportionalGain(0.1) // volts per rotation per second
                );

    public static final double NOTE_AMPS = 40;
  }
}
