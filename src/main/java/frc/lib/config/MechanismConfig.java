package frc.lib.config;

/** Mechanism config. */
public class MechanismConfig {

  /** Absolute encoder config. */
  public AbsoluteEncoderConfig absoluteEncoder = new AbsoluteEncoderConfig();

  /** Feedback controller config. */
  public FeedbackControllerConfig feedback = new FeedbackControllerConfig();

  /** Feedforward controller config. */
  public FeedforwardControllerConfig feedforward = new FeedforwardControllerConfig();

  /** Motor config. */
  public MotorConfig motor = new MotorConfig();

  /**
   * Modifies this mechanism config to use the absolute encoder config.
   *
   * @param absoluteEncoderConfig the absolute encoder config.
   * @return this mechanism config.
   */
  public MechanismConfig withAbsoluteEncoder(AbsoluteEncoderConfig absoluteEncoderConfig) {
    this.absoluteEncoder = absoluteEncoderConfig;
    return this;
  }

  /**
   * Modifies this mechanism config to use the feedback controller config.
   *
   * @param feedbackControllerConfig the feedback controller config.
   * @return this mechanism config.
   */
  public MechanismConfig withFeedback(FeedbackControllerConfig feedbackControllerConfig) {
    this.feedback = feedbackControllerConfig;
    return this;
  }

  /**
   * Modifies this mechanism config to use the feedforward controller config.
   *
   * @param feedforwardControllerConfig the feedforward controller config.
   * @return this mechanism config.
   */
  public MechanismConfig withFeedforward(FeedforwardControllerConfig feedforwardControllerConfig) {
    this.feedforward = feedforwardControllerConfig;
    return this;
  }

  /**
   * Modifies this mechanism config to use the motor config.
   *
   * @param motorConfig the motor config.
   * @return this mechanism config.
   */
  public MechanismConfig withMotor(MotorConfig motorConfig) {
    this.motor = motorConfig;
    return this;
  }
}
