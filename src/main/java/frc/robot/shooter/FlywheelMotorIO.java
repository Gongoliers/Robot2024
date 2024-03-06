package frc.robot.shooter;

/** Flywheel motor hardware interface. */
public interface FlywheelMotorIO {

  /** Values for the flywheel motor hardware interface. */
  public static class FlywheelMotorIOValues {
    /** Velocity of the flywheel in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;

    /** Current drawn by the flywheel motor in amps. */
    public double currentAmps = 0.0;
  }

  /** Configures the flywheel motor. */
  public void configure();

  /**
   * Updates the flywheel motor's values.
   *
   * @param values
   */
  public void update(FlywheelMotorIOValues values);

  /**
   * Sets the setpoint of the flywheel motor.
   *
   * @param velocityRotationsPerSecond the velocity setpoint of the flywheel motor.
   */
  public void setSetpoint(double velocityRotationsPerSecond);
}
