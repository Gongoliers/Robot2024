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
  // TODO Retune to account for change from radians -> rotations
  private final RotationPIDController driftFeedback = new RotationPIDController(0, 0, 0);

  /* Heading feedback controller. */
  // TODO Retune to account for change from radians -> rotations
  private final ProfiledRotationPIDController headingFeedback =
      new ProfiledRotationPIDController(
          8, 0, 0, new Constraints(SwerveConstants.MAXIMUM_ROTATION_SPEED.getRotations(), 0.25));

  private final DoubleEntry headingSetpointEntry,
      headingVelocitySetpointEntry,
      headingVelocityEntry;

  /** Heading setpoint. */
  private Rotation2d headingGoal = new Rotation2d();

  private final DoubleEntry headingEntry, headingGoalEntry;

  public Drive(CustomXboxController driverController) {
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance();

    addRequirements(swerve);

    this.driverController = driverController;
    previousRequest = DriveRequest.fromController(driverController);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("swerve/driveCommand");

    headingSetpointEntry = table.getDoubleTopic("headingSetpoint").getEntry(0);
    headingSetpointEntry.set(0);

    headingVelocitySetpointEntry = table.getDoubleTopic("headingVelocitySetpoint").getEntry(0);
    headingVelocitySetpointEntry.set(0);

    headingVelocityEntry = table.getDoubleTopic("headingVelocity").getEntry(0);
    headingVelocityEntry.set(0);

    headingEntry = table.getDoubleTopic("heading").getEntry(0);
    headingEntry.set(0);

    headingGoalEntry = table.getDoubleTopic("headingGoal").getEntry(0);
    headingGoalEntry.set(0);
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
    }

    Rotation2d rotationVelocity = new Rotation2d();

    switch (request.rotationMode) {
      case SPINNING:
        rotationVelocity = request.getRotationVelocity();
        break;
      case SNAPPING:
        headingGoal = request.getHeading();
        rotationVelocity = headingFeedback.calculate(positionHeading, headingGoal);

        headingGoalEntry.set(headingGoal.getRotations());
        headingSetpointEntry.set(headingFeedback.getSetpoint().position);
        headingVelocitySetpointEntry.set(headingFeedback.getSetpoint().velocity);
        break;
      case DRIFTING:
        rotationVelocity = driftFeedback.calculate(positionHeading, headingGoal);
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
