package frc.robot.swerve;

/** Drive motor hardware interface. */
public interface DriveMotorIO {

  /** Values for the drive motor hardware interface. */
  public static class DriveMotorIOValues {
    /** Position of the drive motor in meters. */
    public double positionMeters = 0.0;

    /** Velocity of the drive motor in meters per second. */
    public double velocityMetersPerSecond = 0.0;
  }

  /** Configures the drive motor. */
  public void configure();

  /**
   * Updates the drive motor's values.
   *
   * @param values
   */
  public void update(DriveMotorIOValues values);

  /**
   * Sets the drive motor's position.
   *
   * @param positionMeters the drive motor's position.
   */
  public void setPosition(double positionMeters);

  /**
   * Sets the drive motor's setpoint.
   *
   * @param velocityMetersPerSecond the drive motor's setpoint.
   */
  public void setSetpoint(double velocityMetersPerSecond);

  /**
   * Sets the drive motor's brake mode.
   *
   * @param brake if true, brake.
   */
  public void setBrake(boolean brake);
}
