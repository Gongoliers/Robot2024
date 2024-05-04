package frc.lib.config;

import edu.wpi.first.math.controller.PIDController;
import java.util.Objects;

/** Feedback controller config. */
public record FeedbackControllerConfig(
    double kP,
    double kI,
    double kD,
    boolean continuous,
    double positionTolerance,
    double velocityTolerance) {

  public FeedbackControllerConfig {
    Objects.requireNonNull(kP);
    Objects.requireNonNull(kI);
    Objects.requireNonNull(kD);
    Objects.requireNonNull(continuous);
    Objects.requireNonNull(positionTolerance);
    Objects.requireNonNull(velocityTolerance);
  }

  public static final class FeedbackControllerConfigBuilder {
    private double kP;

    private double kI;

    private double kD;

    private boolean continuous;

    private double positionTolerance;

    private double velocityTolerance;

    public static FeedbackControllerConfigBuilder defaults() {
      return new FeedbackControllerConfigBuilder(0.0, 0.0, 0.0, false, 0.0, 0.0);
    }

    public static FeedbackControllerConfigBuilder from(
        FeedbackControllerConfig feedbackControllerConfig) {
      return new FeedbackControllerConfigBuilder(
          feedbackControllerConfig.kP,
          feedbackControllerConfig.kI,
          feedbackControllerConfig.kD,
          feedbackControllerConfig.continuous,
          feedbackControllerConfig.positionTolerance,
          feedbackControllerConfig.velocityTolerance);
    }

    private FeedbackControllerConfigBuilder(
        double kP,
        double kI,
        double kD,
        boolean continuous,
        double positionTolerance,
        double velocityTolerance) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.continuous = continuous;
      this.positionTolerance = positionTolerance;
      this.velocityTolerance = velocityTolerance;
    }

    public FeedbackControllerConfigBuilder kP(double kP) {
      this.kP = kP;
      return this;
    }

    public FeedbackControllerConfigBuilder kI(double kI) {
      this.kI = kI;
      return this;
    }

    public FeedbackControllerConfigBuilder kD(double kD) {
      this.kD = kD;
      return this;
    }

    public FeedbackControllerConfigBuilder continuous(boolean continuous) {
      this.continuous = continuous;
      return this;
    }

    public FeedbackControllerConfigBuilder positionTolerance(double positionTolerance) {
      this.positionTolerance = positionTolerance;
      return this;
    }

    public FeedbackControllerConfigBuilder velocityTolerance(double velocityTolerance) {
      this.velocityTolerance = velocityTolerance;
      return this;
    }

    public FeedbackControllerConfig build() {
      return new FeedbackControllerConfig(
          kP, kI, kD, continuous, positionTolerance, velocityTolerance);
    }
  }

  /**
   * Creates a new PID controller using this feedback config.
   *
   * @return a new PID controller using this feedback config.
   */
  public PIDController createPIDController() {
    final PIDController pidController = new PIDController(kP, kI, kD);

    pidController.setTolerance(positionTolerance, velocityTolerance);

    if (continuous) {
      pidController.enableContinuousInput(-0.5, 0.5);
    }

    return pidController;
  }
}
