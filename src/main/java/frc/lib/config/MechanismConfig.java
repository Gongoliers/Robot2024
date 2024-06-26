package frc.lib.config;

import frc.lib.config.AbsoluteEncoderConfig.AbsoluteEncoderConfigBuilder;
import frc.lib.config.FeedbackControllerConfig.FeedbackControllerConfigBuilder;
import frc.lib.config.FeedforwardControllerConfig.FeedforwardControllerConfigBuilder;
import frc.lib.config.MotionProfileConfig.MotionProfileConfigBuilder;
import frc.lib.config.MotorConfig.MotorConfigBuilder;
import java.util.Objects;
import java.util.function.Function;

/** Mechanism config. */
public record MechanismConfig(
    AbsoluteEncoderConfig absoluteEncoderConfig,
    FeedbackControllerConfig feedbackControllerConfig,
    FeedforwardControllerConfig feedforwardControllerConfig,
    MotionProfileConfig motionProfileConfig,
    MotorConfig motorConfig) {

  public MechanismConfig {
    Objects.requireNonNull(absoluteEncoderConfig);
    Objects.requireNonNull(feedbackControllerConfig);
    Objects.requireNonNull(feedforwardControllerConfig);
    Objects.requireNonNull(motionProfileConfig);
    Objects.requireNonNull(motorConfig);
  }

  /** Mechanism config builder. */
  public static final class MechanismConfigBuilder {

    private AbsoluteEncoderConfigBuilder absoluteEncoderConfigBuilder;

    private FeedbackControllerConfigBuilder feedbackControllerConfigBuilder;

    private FeedforwardControllerConfigBuilder feedforwardControllerConfigBuilder;

    private MotionProfileConfigBuilder motionProfileConfigBuilder;

    private MotorConfigBuilder motorConfigBuilder;

    public static MechanismConfigBuilder defaults() {
      return new MechanismConfigBuilder(
          AbsoluteEncoderConfigBuilder.defaults(),
          FeedbackControllerConfigBuilder.defaults(),
          FeedforwardControllerConfigBuilder.defaults(),
          MotionProfileConfigBuilder.defaults(),
          MotorConfigBuilder.defaults());
    }

    public static MechanismConfigBuilder from(MechanismConfig mechanismConfig) {
      return new MechanismConfigBuilder(
          AbsoluteEncoderConfigBuilder.from(mechanismConfig.absoluteEncoderConfig),
          FeedbackControllerConfigBuilder.from(mechanismConfig.feedbackControllerConfig),
          FeedforwardControllerConfigBuilder.from(mechanismConfig.feedforwardControllerConfig),
          MotionProfileConfigBuilder.from(mechanismConfig.motionProfileConfig),
          MotorConfigBuilder.from(mechanismConfig.motorConfig));
    }

    private MechanismConfigBuilder(
        AbsoluteEncoderConfigBuilder absoluteEncoderConfigBuilder,
        FeedbackControllerConfigBuilder feedbackControllerConfigBuilder,
        FeedforwardControllerConfigBuilder feedforwardControllerConfigBuilder,
        MotionProfileConfigBuilder motionProfileConfigBuilder,
        MotorConfigBuilder motorConfigBuilder) {
      this.absoluteEncoderConfigBuilder = absoluteEncoderConfigBuilder;
      this.feedbackControllerConfigBuilder = feedbackControllerConfigBuilder;
      this.feedforwardControllerConfigBuilder = feedforwardControllerConfigBuilder;
      this.motionProfileConfigBuilder = motionProfileConfigBuilder;
      this.motorConfigBuilder = motorConfigBuilder;
    }

    public MechanismConfigBuilder absoluteEncoderConfig(
        Function<AbsoluteEncoderConfigBuilder, AbsoluteEncoderConfigBuilder> modifier) {
      this.absoluteEncoderConfigBuilder = modifier.apply(absoluteEncoderConfigBuilder);
      return this;
    }

    public MechanismConfigBuilder feedbackControllerConfig(
        Function<FeedbackControllerConfigBuilder, FeedbackControllerConfigBuilder> modifier) {
      this.feedbackControllerConfigBuilder = modifier.apply(feedbackControllerConfigBuilder);
      return this;
    }

    public MechanismConfigBuilder feedforwardControllerConfig(
        Function<FeedforwardControllerConfigBuilder, FeedforwardControllerConfigBuilder> modifier) {
      this.feedforwardControllerConfigBuilder = modifier.apply(feedforwardControllerConfigBuilder);
      return this;
    }

    public MechanismConfigBuilder motionProfileConfig(
        Function<MotionProfileConfigBuilder, MotionProfileConfigBuilder> modifier) {
      this.motionProfileConfigBuilder = modifier.apply(motionProfileConfigBuilder);
      return this;
    }

    public MechanismConfigBuilder motorConfig(
        Function<MotorConfigBuilder, MotorConfigBuilder> modifier) {
      this.motorConfigBuilder = modifier.apply(motorConfigBuilder);
      return this;
    }

    /**
     * Returns the built mechanism config.
     *
     * @return the built mechanism config.
     */
    public MechanismConfig build() {
      return new MechanismConfig(
          absoluteEncoderConfigBuilder.build(),
          feedbackControllerConfigBuilder.build(),
          feedforwardControllerConfigBuilder.build(),
          motionProfileConfigBuilder.build(),
          motorConfigBuilder.build());
    }
  }
}
