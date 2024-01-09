package frc.robot.odometry;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;

/** Subsystem class for the odometry subsystem. */
public class Odometry extends Subsystem {

  /** Instance variable for the odometry subsystem singleton. */
  private static Odometry instance = null;

  /** Creates a new instance of the odometry subsystem. */
  private Odometry() {}

  /**
   * Gets the instance of the odometry subsystem.
   *
   * @return the instance of the odometry subsystem.
   */
  public static Odometry getInstance() {
    if (instance == null) {
      instance = new Odometry();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {}
}
