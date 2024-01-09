package frc.robot.swerve;

/** Steer motor hardware interface. */
public interface SteerMotorIO {

  /** Values for the steer motor hardware interface. */
  public static class SteerMotorIOValues {
    /** Position of the steer motor in rotations. */
    public double angleRotations = 0.0;

    /** Velocity of the steer motor in rotations per second. */
    public double omegaRotationsPerSecond = 0.0;
  }

  /**
   * Updates the steer motor's values.
   *
   * @param values
   */
  public void update(SteerMotorIOValues values);

  /**
   * Sets the steer motor's position.
   *
   * @param angleRotations the steer motor's position.
   */
  public void setPosition(double angleRotations);

  /**
   * Sets the steer motor's setpoint.
   *
   * @param angleRotations the steer motor's setpoint.
   */
  public void setSetpoint(double angleRotations);
}
