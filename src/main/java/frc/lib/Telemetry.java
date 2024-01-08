package frc.lib;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Helper class for managing robot telemetry. */
public class Telemetry {

  /**
   * Initializes a subsystem's Shuffleboard tab.
   *
   * @param subsystem the subsystem to initialize.
   */
  public static void initializeShuffleboard(Subsystem subsystem) {
    String name = subsystem.getName();

    ShuffleboardTab tab = Shuffleboard.getTab(name);

    subsystem.addToShuffleboard(tab);
  }
}
