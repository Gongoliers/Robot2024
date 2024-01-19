package frc.lib;

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
}
