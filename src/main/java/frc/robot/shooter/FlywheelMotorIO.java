package frc.robot.shooter;

/** Flywheel motor hardware interface. */
public interface FlywheelMotorIO {

  /** Values for the flywheel motor hardware interface. */
  public static class FlywheelMotorIOValues {
    /** Angular velocity of the flywheel in rotations per second. */
    public double angularVelocityRotationsPerSecond = 0.0;

    /** Current drawn by the flywheel motor in amps. */
    public double currentAmps = 0.0;
  }

  /**
   * Updates the flywheel motor's values.
   *
   * @param values
   */
  public void update(FlywheelMotorIOValues values);

  /**
   * Run the flywheel motor with the specified voltage.
   *
   * @param volts volts to apply to the flywheel motor.
   */
  public void setVoltage(double volts);

  /** Stop the flywheel motor. */
  public void stop();
}
