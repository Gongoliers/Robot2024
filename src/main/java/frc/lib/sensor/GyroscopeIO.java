package frc.lib.sensor;

/** Gyroscope interface. */
public interface GyroscopeIO {

  /** Gyroscope interface. */
  public static class GyroscopeIOValues {
    /** Roll angle in rotations. */
    public double rollRotations = 0.0;

    /** Pitch angle in rotations. */
    public double pitchRotations = 0.0;

    /** Yaw angle in rotations. */
    public double yawRotations = 0.0;

    /** Roll velocity in rotations. */
    public double rollVelocityRotations = 0.0;

    /** Pitch velocity in rotations. */
    public double pitchVelocityRotations = 0.0;

    /** Yaw velocity in rotations. */
    public double yawVelocityRotations = 0.0;
  }

  /** Configures the gyroscope. */
  public void configure();

  /**
   * Updates the gyroscope's values.
   *
   * @param values
   */
  public void update(GyroscopeIOValues values);

  /**
   * Sets the gyroscope's yaw in rotations.
   *
   * @param yawRotations the gyroscope's yaw.
   */
  public void setYaw(double yawRotations);
}
