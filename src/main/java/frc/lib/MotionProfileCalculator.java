package frc.lib;

/** Helper class to assist with motion profile calculations. */
public class MotionProfileCalculator {

  /**
   * Calculates an acceleration using a ramp duration.
   *
   * @param maximumSpeed the maximum speed in units per second.
   * @param desiredRampDurationSeconds the desired duration to ramp from no speed to full speed.
   * @return the acceleration in units per second per second.
   */
  public static double calculateAcceleration(
      double maximumSpeed, double desiredRampDurationSeconds) {
    return maximumSpeed / desiredRampDurationSeconds;
  }
}
