package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

/** Vision pose estimator that does nothing. */
public class VisionPoseEstimatorIONull implements VisionPoseEstimatorIO {

  @Override
  public void configure() {}

  @Override
  public void setReferencePosition(Pose2d referencePosition) {}

  @Override
  public Optional<Pose3d> getPosition() {
    return Optional.empty();
  }
}
