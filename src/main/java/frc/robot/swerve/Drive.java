package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.CustomXboxController;
import frc.lib.RotationPIDController;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.DriveRequest.TranslationMode;

/** Drives the swerve using driver input. */
public class Drive extends Command {
  /* Swerve subsystem. */
  private final Swerve swerve;
  /* Odometry subsystem. */
  private final Odometry odometry;

  /* Xbox controller used to get driver input. */
  private final CustomXboxController controller;

  /* Current and previous requests from the driver controller. */
  private DriveRequest request, previousRequest;

  /* Drift feedback controller. */
  private final RotationPIDController driftFeedback = new RotationPIDController(0, 0, 0);
  /* Heading feedback controller. */
  private final RotationPIDController headingFeedback = new RotationPIDController(0, 0, 0);

  /** Heading setpoint. */
  private Rotation2d heading = new Rotation2d();

  public Drive(CustomXboxController controller) {
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance();

    addRequirements(swerve);

    this.controller = controller;
    previousRequest = DriveRequest.fromController(controller);

    driftFeedback.setSaturation(SwerveConstants.MAXIMUM_ANGULAR_SPEED.times(0.5));
    headingFeedback.setSaturation(SwerveConstants.MAXIMUM_ANGULAR_SPEED);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    request = DriveRequest.fromController(controller);

    Translation2d velocity = request.getTranslationVelocity();

    final Rotation2d rotation = odometry.getPosition().getRotation();

    if (DriveRequest.startedDrifting(previousRequest, request)) {
      heading = rotation;
    }

    double omegaRadiansPerSecond = 0.0;

    switch (request.rotationMode) {
      case SPINNING:
        omegaRadiansPerSecond = request.getSpinRate().getRadians();
        break;
      case SNAPPING:
        heading = request.getRequestedSnapAngle();
        omegaRadiansPerSecond = headingFeedback.calculate(rotation, heading);
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
