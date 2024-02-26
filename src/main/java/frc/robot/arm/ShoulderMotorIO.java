package frc.robot.arm;

/** Shoulder motor hardware interface. */
public interface ShoulderMotorIO {

  /** Values for the shoulder motor hardware interface. */
  public static class ShoulderMotorIOValues {
    /** Position of the shoulder motor in rotations. */
    public double positionRotations = 0.0;

    /** Velocity of the shoulder motor in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;

    /** Acceleration of the shoulder motor in rotations per second per second. */
    public double accelerationRotationsPerSecondPerSecond = 0.0;

    /** Current drawn by the shoulder motor in amps. */
    public double currentAmps = 0.0;

    /** Voltage applied to the shoulder motor in volts. */
    public double inputVoltage = 0.0;
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
   * Runs the shoulder motor's setpoint.
   *
   * @param positionRotations the shoulder motor's position setpoint.
   * @param velocityRotationsPerSecond the shoulder motor's velocity setpoint.
   */
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond);
}
