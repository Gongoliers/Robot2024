package frc.robot.intake;

/** Pivot motor hardware interface. */
public interface PivotMotorIO {

  /** Values for the pivot motor hardware interface. */
  public static class PivotMotorIOValues {
    /** Position of the pivot in rotations. */
    public double positionRotations = 0.0;

    /** Velocity of the pivot in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;
  }

  /** Configures the pivot motor. */
  public void configure();

  /**
   * Updates the pivot motor's values.
   *
   * @param values
   */
  public void update(PivotMotorIOValues values);

  /**
   * Sets the position of the pivot motor.
   *
   * @param positionRotations the position of the pivot motor.
   */
  public void setPosition(double positionRotations);

  /**
   * Sets the setpoint of the pivot motor.
   *
   * @param positionRotations the position setpoint of the pivot motor.
   * @param velocityRotationsPerSecond the velocity setpoint of the pivot motor.
   */
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond);
}
