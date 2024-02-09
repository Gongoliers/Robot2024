package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

/** Helper class to assist with feedforward calculations. */
public class FeedforwardUtil {

  /**
   * Calculates the gravity compensation constant for an arm given the voltage applied at an
   * equilibrium position.
   *
   * @param angle the equilibrium position of the arm.
   * @param volts the voltage applied to the arm to hold it the equilibrium position.
   * @return the gravity compensation constant for the arm.
   */
  public static double calculateArmGravityCompensation(Rotation2d angle, double volts) {
    return volts / angle.getCos();
  }
}
