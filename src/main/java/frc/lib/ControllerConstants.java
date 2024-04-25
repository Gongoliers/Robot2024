package frc.lib;

import frc.lib.config.FeedbackControllerConfig;
import frc.lib.config.FeedforwardControllerConfig;

/** Controller constants. */
public class ControllerConstants {

  /** Feedforward controller constants. */
  public FeedforwardControllerConfig feedforward = new FeedforwardControllerConfig();

  /** Feedback controller constants. */
  public FeedbackControllerConfig feedback = new FeedbackControllerConfig();

  /**
   * Modifies these controller constants to use the feedforward controller constants.
   *
   * @param feedforwardControllerConstants the feedforward controller constants to use.
   * @return these controller constants.
   */
  public ControllerConstants withFeedforward(
      FeedforwardControllerConfig feedforwardControllerConstants) {
    this.feedforward = feedforwardControllerConstants;
    return this;
  }

  /**
   * Modifies these controller constants to use the feedback controller constants.
   *
   * @param feedbackControllerConstants the feedback controller constants to use.
   * @return these controller constants.
   */
  public ControllerConstants withFeedback(FeedbackControllerConfig feedbackControllerConstants) {
    this.feedback = feedbackControllerConstants;
    return this;
  }
}
