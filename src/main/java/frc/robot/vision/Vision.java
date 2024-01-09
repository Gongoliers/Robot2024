package frc.robot.vision;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;

/** Subsystem class for the vision subsystem. */
public class Vision extends Subsystem {

  /** Instance variable for the vision subsystem singleton. */
  private static Vision instance = null;

  /** Creates a new instance of the vision subsystem. */
  private Vision() {}

  /**
   * Gets the instance of the vision subsystem.
   *
   * @return the instance of the vision subsystem.
   */
  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {}

}
