package frc.robot.swerve;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;

/** Subsystem class for the swerve subsystem. */
public class Swerve extends Subsystem {

  /** Instance variable for the swerve subsystem singleton. */
  private static Swerve instance = null;

  /** Creates a new instance of the swerve subsystem. */
  private Swerve() {}

  /**
   * Gets the instance of the swerve subsystem.
   *
   * @return the instance of the swerve subsystem.
   */
  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {}
}
