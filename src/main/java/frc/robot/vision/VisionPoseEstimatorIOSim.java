package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Simulated vision pose estimator. */
public class VisionPoseEstimatorIOSim implements VisionPoseEstimatorIO {

  /** Vision system simulator. */
  private final VisionSystemSim visionSim;

  /** Creates a new simulated vision pose estimator. */
  public VisionPoseEstimatorIOSim() {
    visionSim = new VisionSystemSim("visionSystem");

    try {
      AprilTagFieldLayout tagLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      visionSim.addAprilTags(tagLayout);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    SimCameraProperties cameraProperties = new SimCameraProperties();

    PhotonCamera camera = new PhotonCamera("cameraName");

    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);

    Translation3d robotToCameraTranslation = new Translation3d();

    Rotation3d robotToCameraRotation = new Rotation3d();

    Transform3d robotToCamera = new Transform3d(robotToCameraTranslation, robotToCameraRotation);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void configure() {}

  @Override
  public void setReferencePosition(Pose2d referencePosition) {
    visionSim.update(referencePosition);
  }

  @Override
  public Optional<Pose3d> getPosition() {
    return Optional.of(visionSim.getRobotPose());
  }
}
