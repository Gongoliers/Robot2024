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
  private final CustomXboxController driverController;

  /* Current and previous requests from the driver controller. */
  private DriveRequest request, previousRequest;

  /* Drift feedback controller. */
  private final RotationPIDController driftFeedback = new RotationPIDController(1, 0, 0);

  /* Heading feedback controller. */
  private final RotationPIDController headingFeedback = new RotationPIDController(1, 0, 0);

  /** Heading setpoint. */
  private Rotation2d headingSetpoint = new Rotation2d();

  public Drive(CustomXboxController driverController) {
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance();

    addRequirements(swerve);

    this.driverController = driverController;

    previousRequest = DriveRequest.fromController(driverController);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    request = DriveRequest.fromController(driverController);

    Translation2d translationVelocityMetersPerSecond = request.getTranslationVelocity();

    final Rotation2d positionHeading = odometry.getPosition().getRotation();

    if (DriveRequest.startedDrifting(previousRequest, request)) {
      headingSetpoint = positionHeading;
    }

    Rotation2d rotationVelocity = new Rotation2d();

    switch (request.rotationMode) {
      case SPINNING:
        rotationVelocity = request.getRotationVelocity();
        break;
      case SNAPPING:
        headingSetpoint = request.getHeading();
        rotationVelocity = headingFeedback.calculate(positionHeading, headingSetpoint);
        break;
      case DRIFTING:
        rotationVelocity = driftFeedback.calculate(positionHeading, headingSetpoint);
        break;
    }

    ChassisSpeeds chassisSpeeds;

    if (request.translationMode == TranslationMode.ROBOT_CENTRIC) {
      chassisSpeeds =
          new ChassisSpeeds(
              translationVelocityMetersPerSecond.getX(),
              translationVelocityMetersPerSecond.getY(),
              rotationVelocity.getRadians());
    } else {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationVelocityMetersPerSecond.getX(),
              translationVelocityMetersPerSecond.getY(),
              rotationVelocity.getRadians(),
              positionHeading);
    }

    double maxOmegaRadiansPerSecond = SwerveConstants.MAXIMUM_ROTATION_SPEED.getRadians();

    if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > maxOmegaRadiansPerSecond) {
      chassisSpeeds.omegaRadiansPerSecond =
          Math.signum(chassisSpeeds.omegaRadiansPerSecond) * maxOmegaRadiansPerSecond;
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
