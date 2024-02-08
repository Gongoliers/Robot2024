package frc.robot.arm;

/** Shoulder motor hardware interface. */
public interface ShoulderMotorIO {

  /** Values for the shoulder motor hardware interface. */
  public static class ShoulderMotorIOValues {
    /** Position of the shoulder motor in rotations. */
    public double positionRotations = 0.0;

    /** Current drawn by the shoulder motor in amps. */
    public double currentAmps = 0.0;
  }

  /** Configures the shoulder motor. */
  public void configure();

  /**
   * Updates the shoulder motor's values.
   *
   * @param values
   */
  public void update(ShoulderMotorIOValues values);

  /**
   * Sets the shoulder motor's position.
   *
   * @param positionRotations the shoulder motor's position.
   */
  public void setPosition(double positionRotations);

  /**
   * Sets the shoulder motor's setpoint.
   *
   * @param positionRotations the shoulder motor's setpoint.
   */
  public void setSetpoint(double positionRotations);

  // TODO Remove, only for characterization
  /**
   * Run the shoulder motor with the specified voltage.
   *
   * @param volts volts to apply to the shoulder motor.
   */
  public void setVoltage(double volts);

  /** Stop the shoulder motor. */
  public void stop();
}
