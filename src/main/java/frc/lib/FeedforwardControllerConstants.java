package frc.lib;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Feedforward controller constants. */
public class FeedforwardControllerConstants {
  /** Feedforward controller static gain. */
  private double kS = 0.0;

  /** Feedforward controller gravity gain. */
  private double kG = 0.0;

  /** Feedforward controller velocity gain. */
  private double kV = 0.0;

  /** Feedforward controller acceleration gain. */
  private double kA = 0.0;

  /**
   * Modifies these controller constants' static feedforward.
   *
   * @param kS the static feedforward.
   * @return these controller constants.
   */
  public FeedforwardControllerConstants withStaticFeedforward(double kS) {
    this.kS = kS;
    return this;
  }

  /**
   * Modifies these controller constants' gravity feedforward.
   *
   * @param kG the gravity feedforward.
   * @return these controller constants.
   */
  public FeedforwardControllerConstants withGravityFeedforward(double kG) {
    this.kG = kG;
    return this;
  }

  /**
   * Modifies these controller constants' velocity feedforward.
   *
   * @param kV the velocity feedforward.
   * @return these controller constants.
   */
  public FeedforwardControllerConstants withVelocityFeedforward(double kV) {
    this.kV = kV;
    return this;
  }

  /**
   * Modifies these controller constants' acceleration feedforward.
   *
   * @param kA the acceleration feedforward.
   * @return these controller constants.
   */
  public FeedforwardControllerConstants withAccelerationFeedfoward(double kA) {
    this.kA = kA;
    return this;
  }

  /**
   * Creates a simple motor feedforward using these feedforward constants.
   *
   * @return a simple motor feedforward using these feedforward constants.
   */
  public SimpleMotorFeedforward createSimpleMotorFeedforward() {
    if (kG != 0.0) {
      // TODO Non-zero gravity feedforward means that simple motor feedforward is not best
    }

    return new SimpleMotorFeedforward(kS, kV, kA);
  }

  /**
   * Creates an arm feedforward using these feedforward constants.
   *
   * @return an arm feedforward using these feedforward constants.
   */
  public ArmFeedforward createArmFeedforward() {
    return new ArmFeedforward(kS, kG, kV, kA);
  }
}
