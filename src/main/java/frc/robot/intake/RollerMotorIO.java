package frc.robot.intake;

/** Intake motor hardware interface. */
public interface RollerMotorIO {

  /** Values for the roller motor hardware interface. */
  public static class RollerMotorIOValues {
    /** Angular velocity of the flywheel in rotations per second. */
    public double angularVelocityRotationsPerSecond = 0.0;

    /** Current drawn by the flywheel motor in amps. */
    public double currentAmps = 0.0;
  }

  /** Configures the roller motor. */
  public void configure();

  /**
   * Updates the roller motor's values.
   *
   * @param values
   */
  public void update(RollerMotorIOValues values);

  /**
   * Run the roller motor with the specified voltage.
   *
   * @param volts the voltage to apply to the voltage motor.
   */
  public void setVoltage(double volts);

  /** Stops the roller motor. */
  public void stop();
}
