package frc.lib;

import com.ctre.phoenix6.configs.Slot0Configs;

/** Constants for PID feedback controllers and feedforward controllers. */
public class PIDFConstants {

  /** Feedback controller proportional gain. */
  public double kP = 0.0;

  /** Feedback controller integral gain. */
  public double kI = 0.0;

  /** Feedback controller derivative gain. */
  public double kD = 0.0;

  /** Feedback controller position tolerance. */
  public double kPositionTolerance = 0.0;

  /** Feedback controller velocity constraint. */
  public double kVelocityConstraint = 0.0;

  /** Feedback controller acceleration constraint. */
  public double kAccelerationConstraint = 0.0;

  /** Feedforward controller static gain. */
  public double kS = 0.0;

  /** Feedforward controller velocity gain. */
  public double kV = 0.0;

  /**
   * Creates a Phoenix PIDF configuration using the PIDF constants.
   *
   * @return a Phoenix PIDF configuration.
   */
  public Slot0Configs asSlot0Configs() {
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    return slot0Configs;
  }
}
