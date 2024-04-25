package frc.lib;

/** Controller constants. */
public class ControllerConstants {

  /** Feedforward controller constants. */
  public FeedforwardControllerConstants feedforward = new FeedforwardControllerConstants();

  /** Feedback controller constants. */
  public FeedbackControllerConstants feedback = new FeedbackControllerConstants();

  /**
   * Modifies these controller constants to use the feedforward controller constants.
   *
   * @param feedforwardControllerConstants the feedforward controller constants to use.
   * @return these controller constants.
   */
  public ControllerConstants withFeedforward(
      FeedforwardControllerConstants feedforwardControllerConstants) {
    this.feedforward = feedforwardControllerConstants;
    return this;
  }

  /**
   * Modifies these controller constants to use the feedback controller constants.
   *
   * @param feedbackControllerConstants the feedback controller constants to use.
   * @return these controller constants.
   */
  public ControllerConstants withFeedback(FeedbackControllerConstants feedbackControllerConstants) {
    this.feedback = feedbackControllerConstants;
    return this;
  }
}
