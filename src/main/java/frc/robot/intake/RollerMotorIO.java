package frc.robot.intake;

/** Intake motor hardware interface. */
public interface RollerMotorIO {

  /** Values for the roller motor hardware interface. */
  public static class RollerMotorIOValues {
    /** Velocity of the roller motor in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;

    /** Current draw of the roller motor in amps. */
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
   * Sets the setpoint of the roller motor.
   *
   * @param velocityRotationsPerSecond the velocity setpoint of the roller motor.
   */
  public void setSetpoint(double velocityRotationsPerSecond);
}
