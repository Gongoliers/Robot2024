package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Helper class to assist with flipping positions behind on alliance. */
public class AllianceFlipHelper {
  /**
   * Determines whether a path should be flipped.
   *
   * @return whether the path should be flipped.
   */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
