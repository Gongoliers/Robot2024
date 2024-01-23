package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

/** Vision pose estimator hardware interface. */
public interface VisionPoseEstimatorIO {

  /** Configures the vision pose estimator. */
  public void configure();

  /**
   * Sets the vision pose estimator's reference position.
   *
   * @param referencePosition the reference position.
   */
  public void setReferencePosition(Pose2d referencePosition);

  /**
   * Gets the estimated position.
   *
   * @return the estimated position.
   */
  public Optional<Pose3d> getPosition();
}