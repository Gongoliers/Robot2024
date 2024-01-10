package frc.robot.shooter;

/** Beam break sensor hardware interface. */
public interface BeamBreakSensorIO {

  /** Values for the beam break sensor hardware interface. */
  public static class BeamBreakSensorIOValues {
    /** If true, the beam break sensor is broken. */
    public boolean isBroken = false;
  }

  /** Configures the beam break sensor. */
  public void configure();

  /**
   * Updates the beam break sensor's values.
   *
   * @param values
   */
  public void update(BeamBreakSensorIOValues values);
}
