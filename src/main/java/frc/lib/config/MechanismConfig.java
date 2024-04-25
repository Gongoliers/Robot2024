package frc.lib.config;

/** Mechanism config. */
public class MechanismConfig {

  /** Absolute encoder config. */
  private AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();

  /** Feedback controller config. */
  private FeedbackControllerConfig feedbackControllerConfig = new FeedbackControllerConfig();

  /** Feedforward controller config. */
  private FeedforwardControllerConfig feedforwardControllerConfig =
      new FeedforwardControllerConfig();

  /** Motor config. */
  private MotorConfig motorConfig = new MotorConfig();

  /**
   * Modifies this mechanism config to use the absolute encoder config.
   *
   * @param absoluteEncoderConfig the absolute encoder config.
   * @return this mechanism config.
   */
  public MechanismConfig withAbsoluteEncoderConfig(AbsoluteEncoderConfig absoluteEncoderConfig) {
    this.absoluteEncoderConfig = absoluteEncoderConfig;
    return this;
  }

  /**
   * Modifies this mechanism config to use the feedback controller config.
   *
   * @param feedbackControllerConfig the feedback controller config.
   * @return this mechanism config.
   */
  public MechanismConfig withFeedbackConfig(FeedbackControllerConfig feedbackControllerConfig) {
    this.feedbackControllerConfig = feedbackControllerConfig;
    return this;
  }

  /**
   * Modifies this mechanism config to use the feedforward controller config.
   *
   * @param feedforwardControllerConfig the feedforward controller config.
   * @return this mechanism config.
   */
  public MechanismConfig withFeedforwardConfig(
      FeedforwardControllerConfig feedforwardControllerConfig) {
    this.feedforwardControllerConfig = feedforwardControllerConfig;
    return this;
  }

  /**
   * Modifies this mechanism config to use the motor config.
   *
   * @param motorConfig the motor config.
   * @return this mechanism config.
   */
  public MechanismConfig withMotorConfig(MotorConfig motorConfig) {
    this.motorConfig = motorConfig;
    return this;
  }

  /**
   * Returns the absolute encoder config.
   *
   * @return the absolute encoder config.
   */
  public AbsoluteEncoderConfig absoluteEncoderConfig() {
    return absoluteEncoderConfig;
  }

  /**
   * Returns the feedback controller config.
   *
   * @return the feedback controller config.
   */
  public FeedbackControllerConfig feedbackControllerConfig() {
    return feedbackControllerConfig;
  }

  /**
   * Returns the feedforward controller config.
   *
   * @return the feedforward controller config.
   */
  public FeedforwardControllerConfig feedforwardControllerConfig() {
    return feedforwardControllerConfig;
  }

  /**
   * Returns the motor config.
   *
   * @return the motor config.
   */
  public MotorConfig motorConfig() {
    return motorConfig;
  }
}
