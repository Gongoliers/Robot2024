package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.CustomXboxController;
import frc.lib.ProfiledRotationPIDController;
import frc.lib.RotationPIDController;
import frc.lib.Telemetry;
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
  // TODO
  private final RotationPIDController driftFeedback = new RotationPIDController(0, 0, 0);

  /* Heading feedback controller. */
  private final ProfiledRotationPIDController headingFeedback =
      new ProfiledRotationPIDController(
          6,
          0,
          0.1,
          new Constraints(
              SwerveConstants.MAXIMUM_ROTATION_SPEED,
              SwerveConstants.MAXIMUM_ROTATION_ACCELERATION));

  private final DoubleEntry headingEntry,
      headingVelocityEntry,
      headingGoalEntry,
      headingSetpointEntry,
      headingVelocitySetpointEntry;

  /** Heading setpoint. */
  private Rotation2d headingGoal = new Rotation2d();

  public Drive(CustomXboxController driverController) {
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance();

    addRequirements(swerve);

    this.driverController = driverController;
    previousRequest = DriveRequest.fromController(driverController);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("swerve/driveCommand");

    headingEntry = Telemetry.addDoubleEntry(table, "heading");
    headingVelocityEntry = Telemetry.addDoubleEntry(table, "headingVelocity");

    headingGoalEntry = Telemetry.addDoubleEntry(table, "headingGoal");

    headingSetpointEntry = Telemetry.addDoubleEntry(table, "headingSetpoint");
    headingVelocitySetpointEntry = Telemetry.addDoubleEntry(table, "headingVelocitySetpoint");
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    request = DriveRequest.fromController(driverController);

    Translation2d translationVelocityMetersPerSecond = request.getTranslationVelocity();

    final Rotation2d positionHeading = odometry.getPosition().getRotation();

    headingEntry.set(positionHeading.getRotations());

    if (DriveRequest.startedDrifting(previousRequest, request)) {
      headingGoal = positionHeading;
      headingGoalEntry.set(headingGoal.getRotations());
    }

    double rotationVelocityRotationsPerSecond = 0.0;

    switch (request.rotationMode) {
      case SPINNING:
        rotationVelocityRotationsPerSecond = request.getRotationVelocity();
        break;
      case SNAPPING:
        headingGoal = request.getHeading();
        rotationVelocityRotationsPerSecond =
            headingFeedback.calculate(positionHeading, headingGoal);

        headingGoalEntry.set(headingGoal.getRotations());
        headingSetpointEntry.set(headingFeedback.getSetpoint().position);
        headingVelocitySetpointEntry.set(headingFeedback.getSetpoint().velocity);
        break;
      case DRIFTING:
        rotationVelocityRotationsPerSecond = driftFeedback.calculate(positionHeading, headingGoal);
        break;
    }

    ChassisSpeeds chassisSpeeds;

    if (request.translationMode == TranslationMode.ROBOT_CENTRIC) {
      chassisSpeeds =
          new ChassisSpeeds(
              translationVelocityMetersPerSecond.getX(),
              translationVelocityMetersPerSecond.getY(),
              Units.rotationsToRadians(rotationVelocityRotationsPerSecond));
    } else {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationVelocityMetersPerSecond.getX(),
              translationVelocityMetersPerSecond.getY(),
              Units.rotationsToRadians(rotationVelocityRotationsPerSecond),
              positionHeading);
    }

    double maxOmegaRadiansPerSecond =
        Units.rotationsToRadians(SwerveConstants.MAXIMUM_ATTAINABLE_ROTATION_SPEED);

    // Clamp angular velocity
    chassisSpeeds.omegaRadiansPerSecond =
        Math.signum(chassisSpeeds.omegaRadiansPerSecond)
            * Math.min(maxOmegaRadiansPerSecond, Math.abs(chassisSpeeds.omegaRadiansPerSecond));

    headingVelocityEntry.set(Units.radiansToRotations(chassisSpeeds.omegaRadiansPerSecond));

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
