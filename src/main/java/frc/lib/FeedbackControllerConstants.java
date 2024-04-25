package frc.lib;

import edu.wpi.first.math.controller.PIDController;

/** Feedback controller constants. */
public class FeedbackControllerConstants {
  /** Feedback controller proportional gain. */
  private double kP = 0.0;

  /** Feedback controller integral gain. */
  private double kI = 0.0;

  /** Feedback controller derivative gain. */
  private double kD = 0.0;

  /** Feedback controller position tolerance. */
  private double kPositionTolerance = 0.0;

  /** Feedback controller velocity tolerance. */
  private double kVelocityTolerance = 0.0;

  // TODO Add velocity / acceleration constraints, profiled PID controller?

  /**
   * Modifies these controller constants' proportional gain.
   *
   * @param kP the proportional gain.
   * @return these controller constants.
   */
  public FeedbackControllerConstants withProportionalGain(double kP) {
    this.kP = kP;
    return this;
  }

  /**
   * Modifies these controller constants' integral gain.
   *
   * @param kI the integral gain.
   * @return these controller constants.
   */
  public FeedbackControllerConstants withIntegralGain(double kI) {
    this.kI = kI;
    return this;
  }

  /**
   * Modifies these controller constants' derivative gain.
   *
   * @param kD the derivative gain.
   * @return these controller constants.
   */
  public FeedbackControllerConstants withDerivativeGain(double kD) {
    this.kD = kD;
    return this;
  }

  /**
   * Modifies these controller constants' position tolerance.
   *
   * @param kPositionTolerance the position tolerance.
   * @return these controller constants.
   */
  public FeedbackControllerConstants withPositionTolerance(double kPositionTolerance) {
    this.kPositionTolerance = kPositionTolerance;
    return this;
  }

  /**
   * Modifies these controller constants' velocity tolerance.
   *
   * @param kVelocityTolerance the velocity tolerance.
   * @return these controller constants.
   */
  public FeedbackControllerConstants withVelocityTolerance(double kVelocityTolerance) {
    this.kVelocityTolerance = kVelocityTolerance;
    return this;
  }

  /**
   * Creates a new PID controller using these feedback constants.
   *
   * @return a new PID controller using these feedback constants.
   */
  public PIDController createPIDController() {
    PIDController pidController = new PIDController(kP, kI, kD);

    pidController.setTolerance(kPositionTolerance, kVelocityTolerance);

    return pidController;
  }
}
