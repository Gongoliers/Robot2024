package frc.robot.shooter;

/** Serializer motor hardware interface. */
public interface SerializerMotorIO {

  /** Values for the serializer motor hardware interface. */
  public static class SerializerMotorIOValues {
    /** Angular velocity of the serializer in rotations per second. */
    public double angularVelocityRotationsPerSecond = 0.0;

    /** Current drawn by the serializer motor in amps. */
    public double currentAmps = 0.0;
  }

  /**
   * Updates the serializer motor's values.
   *
   * @param values
   */
  public void update(SerializerMotorIOValues values);

  /**
   * Run the serializer motor with the specified voltage.
   *
   * @param volts volts to apply to the serializer motor.
   */
  public void setVoltage(double volts);

  /** Stop the serializer motor. */
  public void stop();
}
