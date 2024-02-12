package frc.robot.vision;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the vision subsystem. */
public class VisionFactory {

  /**
   * Creates a vision pose estimator.
   *
   * @return a vision pose estimator.
   */
  public static VisionPoseEstimatorIO createVisionPoseEstimator() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.VISION))
      return new VisionPoseEstimatorIONull();

    return new VisionPoseEstimatorIOSim();
  }
}
