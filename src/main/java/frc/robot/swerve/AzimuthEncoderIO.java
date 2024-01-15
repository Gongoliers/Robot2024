package frc.robot.swerve;

/** Azimuth encoder hardware interface. */
public interface AzimuthEncoderIO {

  /** Values for the azimuth encoder hardware interface. */
  public static class AzimuthEncoderIOValues {
    /** Position of the azimuth encoder in rotations. */
    public double positionRotations = 0.0;
  }

  /** Configures the azimuth encoder. */
  public void configure();

  /**
   * Updates the azimuth encoder's values.
   *
   * @param values
   */
  public void update(AzimuthEncoderIOValues values);
}
