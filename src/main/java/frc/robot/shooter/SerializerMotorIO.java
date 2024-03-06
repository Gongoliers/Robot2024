package frc.robot.shooter;

/** Serializer motor hardware interface. */
public interface SerializerMotorIO {

  /** Values for the serializer motor hardware interface. */
  public static class SerializerMotorIOValues {
    /** Velocity of the serializer in rotations per second. */
    public double velocityRotationsPerSecond = 0.0;

    /** Current drawn by the serializer motor in amps. */
    public double currentAmps = 0.0;
  }

  /** Configures the serializer motor. */
  public void configure();

  /**
   * Updates the serializer motor's values.
   *
   * @param values
   */
  public void update(SerializerMotorIOValues values);

  /**
   * Sets the setpoint of the serializer motor.
   *
   * @param velocityRotationsPerSecond the velocity setpoint of the serializer motor.
   */
  public void setSetpoint(double velocityRotationsPerSecond);
}
