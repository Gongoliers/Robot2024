package frc.lib.config;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import java.util.Objects;

/** Feedforward controller config. */
public record FeedforwardControllerConfig(double kS, double kG, double kV, double kA) {

  public FeedforwardControllerConfig {
    Objects.requireNonNull(kS);
    Objects.requireNonNull(kG);
    Objects.requireNonNull(kV);
    Objects.requireNonNull(kA);
  }

  public static final class FeedforwardControllerConfigBuilder {
    private double kS;

    private double kG;

    private double kV;

    private double kA;

    public static FeedforwardControllerConfigBuilder defaults() {
      return new FeedforwardControllerConfigBuilder(0.0, 0.0, 0.0, 0.0);
    }

    public static FeedforwardControllerConfigBuilder from(
        FeedforwardControllerConfig feedforwardControllerConfig) {
      return new FeedforwardControllerConfigBuilder(
          feedforwardControllerConfig.kS,
          feedforwardControllerConfig.kG,
          feedforwardControllerConfig.kV,
          feedforwardControllerConfig.kA);
    }

    private FeedforwardControllerConfigBuilder(double kS, double kG, double kV, double kA) {
      this.kS = kS;
      this.kG = kG;
      this.kV = kV;
      this.kA = kA;
    }

    public FeedforwardControllerConfigBuilder kS(double kS) {
      this.kS = kS;
      return this;
    }

    public FeedforwardControllerConfigBuilder kG(double kG) {
      this.kG = kG;
      return this;
    }

    public FeedforwardControllerConfigBuilder kV(double kV) {
      this.kV = kV;
      return this;
    }

    public FeedforwardControllerConfigBuilder kA(double kA) {
      this.kA = kA;
      return this;
    }

    public FeedforwardControllerConfig build() {
      return new FeedforwardControllerConfig(kS, kG, kV, kA);
    }
  }

  /**
   * Creates a simple motor feedforward using this feedforward config.
   *
   * @return a simple motor feedforward using this feedforward config.
   */
  public SimpleMotorFeedforward createSimpleMotorFeedforward() {
    if (kG != 0.0) {
      // TODO Non-zero gravity feedforward means that simple motor feedforward is not best
    }

    return new SimpleMotorFeedforward(kS, kV, kA);
  }

  /**
   * Creates an arm feedforward using this feedforward config.
   *
   * @return an arm feedforward using this feedforward config.
   */
  public ArmFeedforward createArmFeedforward() {
    return new ArmFeedforward(kS, kG, kV, kA);
  }
}
