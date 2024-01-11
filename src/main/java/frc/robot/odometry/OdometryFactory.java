package frc.robot.odometry;

import frc.robot.Robot;

/** Helper class for creating hardware for the odometry subsystem. */
public class OdometryFactory {

  /**
   * Creates a gyroscope.
   *
   * @return a gyroscope.
   */
  public static GyroscopeIO createGyroscope() {
    if (Robot.isReal()) return new GyroscopeIOSim();

    return new GyroscopeIOSim();
  }
}
