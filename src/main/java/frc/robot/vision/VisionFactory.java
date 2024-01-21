package frc.robot.vision;

import frc.robot.Robot;

/** Helper class for creating hardware for the vision subsystem. */
public class VisionFactory {

  /**
   * Creates a vision pose estimator.
   *
   * @return a vision pose estimator.
   */
  public static VisionPoseEstimatorIO createVisionPoseEstimator() {
    if (Robot.isReal()) return new VisionPoseEstimatorIOSim(); // TODO

    return new VisionPoseEstimatorIOSim();
  }
}
