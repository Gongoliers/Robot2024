package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.CustomXboxController;
import frc.lib.RotationPIDController;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.DriveRequest.TranslationMode;

/** Commands the swerve to drive using driver input. */
public class Drive extends Command {
  /* Reference to the swerve subsystem. */
  private final Swerve swerve;
  /* Reference to the odometry subsystem. */
  private final Odometry odometry;

  /* Reference to the Xbox controller used to get driver input. */
  private final CustomXboxController controller;

  /* Stores the current and previous requests from the driver controller. */
  private DriveRequest request, previousRequest;

  /* Rotation feedback controller. Outputs angular velocities to correct positional error caused by unintentional drift. */
  private final RotationPIDController driftFeedback = new RotationPIDController(0, 0, 0);
  /* Rotation feedback controller. Outputs angular velocities to approach requested headings. */
  private final RotationPIDController thetaFeedback = new RotationPIDController(0, 0, 0);

  private Rotation2d heading = new Rotation2d();

  public Drive(CustomXboxController controller) {
    this.swerve = Swerve.getInstance();
    this.odometry = Odometry.getInstance();

    addRequirements(this.swerve);

    this.controller = controller;
    this.previousRequest = DriveRequest.fromController(controller);

    driftFeedback.setSaturation(SwerveConstants.MAXIMUM_ANGULAR_SPEED.times(0.5));
    thetaFeedback.setSaturation(SwerveConstants.MAXIMUM_ANGULAR_SPEED);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    request = DriveRequest.fromController(controller);

    Translation2d velocity = request.getRequestedVelocity();

    final Rotation2d rotation = odometry.getPosition().getRotation();

    if (DriveRequest.startedDrifting(previousRequest, request)) {
      heading = rotation;
    }

    double omegaRadiansPerSecond = 0.0;

    switch (request.rotationMode) {
      case SPINNING:
        omegaRadiansPerSecond = request.getRequestedSpinRate().getRadians();
        break;
      case SNAPPING:
        heading = request.getRequestedSnapAngle();
        omegaRadiansPerSecond = thetaFeedback.calculate(rotation, heading);
        break;
      case DRIFTING:
        omegaRadiansPerSecond = driftFeedback.calculate(rotation, heading);
        break;
    }

    ChassisSpeeds chassisSpeeds;

    if (request.translationMode == TranslationMode.ROBOT_CENTRIC) {
      chassisSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), omegaRadiansPerSecond);
    } else {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              velocity.getX(), velocity.getY(), omegaRadiansPerSecond, rotation);
    }

    swerve.setChassisSpeeds(chassisSpeeds);

    previousRequest = request;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}