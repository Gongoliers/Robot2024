package frc.lib.config;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Feedforward controller config. */
public class FeedforwardControllerConfig {
  /** Feedforward controller static gain. */
  private double kS = 0.0;

  /** Feedforward controller gravity gain. */
  private double kG = 0.0;

  /** Feedforward controller velocity gain. */
  private double kV = 0.0;

  /** Feedforward controller acceleration gain. */
  private double kA = 0.0;

  /**
   * Modifies this controller config's static feedforward.
   *
   * @param kS the static feedforward.
   * @return this controller config.
   */
  public FeedforwardControllerConfig withStaticFeedforward(double kS) {
    this.kS = kS;
    return this;
  }

  /**
   * Modifies this controller config's gravity feedforward.
   *
   * @param kG the gravity feedforward.
   * @return this controller config.
   */
  public FeedforwardControllerConfig withGravityFeedforward(double kG) {
    this.kG = kG;
    return this;
  }

  /**
   * Modifies this controller config's velocity feedforward.
   *
   * @param kV the velocity feedforward.
   * @return this controller config.
   */
  public FeedforwardControllerConfig withVelocityFeedforward(double kV) {
    this.kV = kV;
    return this;
  }

  /**
   * Modifies this controller config's acceleration feedforward.
   *
   * @param kA the acceleration feedforward.
   * @return this controller config.
   */
  public FeedforwardControllerConfig withAccelerationFeedfoward(double kA) {
    this.kA = kA;
    return this;
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

  /**
   * Returns the feedforward controller static gain.
   *
   * @return the feedforward controller static gain.
   */
  public double kS() {
    return kS;
  }

  /**
   * Returns the feedforward controller gravity gain.
   *
   * @return the feedforward controller gravity gain.
   */
  public double kG() {
    return kG;
  }

  /**
   * Returns the feedforward controller velocity gain.
   *
   * @return the feedforward controller velocity gain.
   */
  public double kV() {
    return kV;
  }

  /**
   * Returns the feedforward controller acceleration gain.
   *
   * @return the feedforward controller acceleration gain.
   */
  public double kA() {
    return kA;
  }
}
