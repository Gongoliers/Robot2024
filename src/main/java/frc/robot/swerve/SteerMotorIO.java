package frc.robot.swerve;

/** Steer motor hardware interface. */
public interface SteerMotorIO {

  /** Values for the steer motor hardware interface. */
  public static class SteerMotorIOValues {
    /** Position of the steer motor in rotations. */
    public double positionRotations = 0.0;

    /** Velocity of the steer motor in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;
  }

  /** Configures the steer motor. */
  public void configure();

  /**
   * Updates the steer motor's values.
   *
   * @param values
   */
  public void update(SteerMotorIOValues values);

  /**
   * Sets the steer motor's position.
   *
   * @param positionRotations the steer motor's position.
   */
  public void setPosition(double positionRotations);

  /**
   * Runs the steer motor's setpoint.
   *
   * @param positionRotations the steer motor's setpoint.
   */
  public void runSetpoint(double positionRotations);
}
