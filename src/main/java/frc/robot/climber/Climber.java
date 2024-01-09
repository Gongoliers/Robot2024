package frc.robot.climber;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;

/** Subsystem class for the intake subsystem. */
public class Climber extends Subsystem {

  /** Instance variable for the intake subsystem singleton. */
  private static Climber instance = null;

  /** Creates a new instance of the climber subsystem. */
  private Climber() {}

  /**
   * Gets the instance of the climber subsystem.
   *
   * @return the instance of the climber subsystem.
   */
  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {}

}
