package frc.robot.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.PoseEstimate;
import java.util.Optional;

public class Limelight {

  private final String name;

  public Limelight(String name) {
    this.name = name;
  }

  public void setTagFilter(int[] tags) {
    LimelightHelpers.SetFiducialIDFiltersOverride(this.name, tags);
  }

  public void setYaw(Rotation2d yaw) {
    LimelightHelpers.SetRobotOrientation(this.name, yaw.getDegrees(), 0, 0, 0, 0, 0);
  }

  public Optional<PoseEstimate> getPoseEstimate() {
    LimelightHelpers.PoseEstimate megaTag2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.name);

    if (megaTag2.tagCount < 1) return Optional.empty();

    return Optional.of(megaTag2);
  }
}
