package frc.robot.arm;

/** Elbow motor hardware interface. */
public interface WristMotorIO {
  /** Values for the elbow motor hardware interface. */
  public static class WristMotorIOValues {
    /** Position of the elbow motor in rotations. */
    public double positionRotations = 0.0;

    /** Velocity of the elbow motor in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;

    /** Acceleration of the elbow motor in rotations per second per second. */
    public double accelerationRotationsPerSecondPerSecond = 0.0;

    /** Current drawn by the elbow motor in amps. */
    public double currentAmps = 0.0;

    /** Voltage applied to the elbow motor in volts. */
    public double inputVoltage = 0.0;
  }

  /** Configures the elbow motor. */
  public void configure();

  /**
   * Updates the elbow motor's values.
   *
   * @param values
   */
  public void update(WristMotorIOValues values);

  /**
   * Sets the elbow motor's position.
   *
   * @param positionRotations the elbow motor's position.
   */
  public void setPosition(double positionRotations);

  /**
   * Runs the elbow motor's setpoint.
   *
   * @param positionRotations the elbow motor's setpoint.
   */
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond);
}
