package frc.lib.config;

import edu.wpi.first.math.controller.PIDController;

/** Feedback controller config. */
public class FeedbackControllerConfig {

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
   * Modifies this controller config's proportional gain.
   *
   * @param kP the proportional gain.
   * @return this controller config.
   */
  public FeedbackControllerConfig withProportionalGain(double kP) {
    this.kP = kP;
    return this;
  }

  /**
   * Modifies this controller config's integral gain.
   *
   * @param kI the integral gain.
   * @return this controller config.
   */
  public FeedbackControllerConfig withIntegralGain(double kI) {
    this.kI = kI;
    return this;
  }

  /**
   * Modifies this controller config's derivative gain.
   *
   * @param kD the derivative gain.
   * @return this controller config.
   */
  public FeedbackControllerConfig withDerivativeGain(double kD) {
    this.kD = kD;
    return this;
  }

  /**
   * Modifies this controller config's position tolerance.
   *
   * @param kPositionTolerance the position tolerance.
   * @return this controller config.
   */
  public FeedbackControllerConfig withPositionTolerance(double kPositionTolerance) {
    this.kPositionTolerance = kPositionTolerance;
    return this;
  }

  /**
   * Modifies this controller config's velocity tolerance.
   *
   * @param kVelocityTolerance the velocity tolerance.
   * @return this controller config.
   */
  public FeedbackControllerConfig withVelocityTolerance(double kVelocityTolerance) {
    this.kVelocityTolerance = kVelocityTolerance;
    return this;
  }

  /**
   * Creates a new PID controller using this feedback config.
   *
   * @return a new PID controller using this feedback config.
   */
  public PIDController createPIDController() {
    PIDController pidController = new PIDController(kP, kI, kD);

    pidController.setTolerance(kPositionTolerance, kVelocityTolerance);

    return pidController;
  }

  /**
   * Returns the feedback controller proportional gain.
   * 
   * @return the feedback controller proportional gain.
    */
  public double kP() {
    return kP;
  }

  /**
   * Returns the feedback controller integral gain.
   * 
   * @return the feedback controller integral gain.
    */
  public double kI() {
    return kI;
  }

  /**
   * Returns the feedback controller derivative gain.
   * 
   * @return the feedback controller derivative gain.
    */
  public double kD() {
    return kD;
  }

  /**
   * Returns the feedback controller position tolerance.
   * 
   * @return the feedback controller position tolerance.
    */
  public double kPositionTolerance() {
    return kPositionTolerance;
  }

  /**
   * Returns the feedback controller velocity tolerance.
   * 
   * @return the feedback controller velocity tolerance.
    */
  public double kVelocityTolerance() {
    return kVelocityTolerance;
  }
}
