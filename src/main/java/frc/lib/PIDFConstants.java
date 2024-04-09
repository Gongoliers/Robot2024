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

  /** Feedback controller velocity tolerance. */
  public double kVelocityTolerance = 0.0;

  /** Feedforward controller static gain. */
  public double kS = 0.0;

  /** Feedforward controller gravity gain. */
  public double kG = 0.0;

  /** Feedforward controller velocity gain. */
  public double kV = 0.0;

  /** Feedforward controller acceleration gain. */
  public double kA = 0.0;

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
    slot0Configs.kG = kG;
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;

    return slot0Configs;
  }
}
