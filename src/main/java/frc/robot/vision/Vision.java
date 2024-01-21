package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.odometry.Odometry;

/** Subsystem class for the vision subsystem. */
public class Vision extends Subsystem {

  /** Instance variable for the vision subsystem singleton. */
  private static Vision instance = null;

  /** Vision pose estimator. */
  private final VisionPoseEstimatorIO visionPoseEstimator;

  /** Creates a new instance of the vision subsystem. */
  private Vision() {
    visionPoseEstimator = VisionFactory.createVisionPoseEstimator();
  }

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
  public void periodic() {
    visionPoseEstimator.setReferencePosition(Odometry.getInstance().getPosition());
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout pose = Telemetry.addColumn(tab, "Vision Pose");

    pose.addDouble("X (m)", () -> visionPoseEstimator.getPosition().orElse(new Pose3d()).getX());
    pose.addDouble("Y (m)", () -> visionPoseEstimator.getPosition().orElse(new Pose3d()).getY());
    pose.addDouble(
        "Rotation (deg)",
        () -> visionPoseEstimator.getPosition().orElse(new Pose3d()).getRotation().getZ());
  }
}
