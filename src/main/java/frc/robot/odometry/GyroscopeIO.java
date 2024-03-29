package frc.robot.odometry;

/** Gyroscope hardware interface. */
public interface GyroscopeIO {

  /** Gyroscope's hardware interface. */
  public static class GyroscopeIOValues {
    /** Roll angle in rotations. */
    public double rollRotations = 0.0;

    /** Pitch angle in rotations. */
    public double pitchRotations = 0.0;

    /** Yaw angle in rotations. */
    public double yawRotations = 0.0;
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
