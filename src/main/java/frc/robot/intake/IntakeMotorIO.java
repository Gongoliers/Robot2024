package frc.robot.intake;

/** Intake motor hardware interface. */
public interface IntakeMotorIO {

  /** Values for the intake motor hardware interface. */
  public static class IntakeMotorIOValues {
    /** Angular velocity of the flywheel in rotations per second. */
    public double angularVelocityRotationsPerSecond = 0.0;

    /** Current drawn by the flywheel motor in amps. */
    public double currentAmps = 0.0;
  }

  /** Configures the intake motor. */
  public void configure();

  /**
   * Updates the intake motor's values/
   *
   * @param values
   */
  public void update(IntakeMotorIOValues values);

  /**
   * Run the intake motor with the specified voltage.
   *
   * @param volts the voltage to apply to the voltage motor.
   */
  public void setVoltage(double volts);

  /** Stops the intake motor. */
  public void stop();
}
