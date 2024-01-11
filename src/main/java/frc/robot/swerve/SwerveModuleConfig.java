package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Objects;

/** Configuration of a swerve module. */
public record SwerveModuleConfig(
    SwerveModuleCAN moduleCAN, Translation2d position, Rotation2d offset) {

  /**
   * Creates the configuration of a swerve module.
   *
   * @param moduleCAN the swerve module's CAN identifiers.
   * @param position the swerve module's position relative to the center of the robot.
   * @param offset the swerve module steer motor's offset.
   */
  public SwerveModuleConfig {
    Objects.requireNonNull(moduleCAN);
    Objects.requireNonNull(position);
    Objects.requireNonNull(offset);
  }
}
