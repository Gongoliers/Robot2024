package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.ProfiledRotationPIDController;
import frc.lib.RotationPIDController;
import frc.lib.Telemetry;
import frc.robot.RobotConstants;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.DriveRequest.TranslationMode;

/** Drives the swerve using driver input. */
public class DriveCommand extends Command {
  /* Swerve subsystem. */
  private final Swerve swerve;
  /* Odometry subsystem. */
  private final Odometry odometry;

  /* Xbox controller used to get driver input. */
  private final CommandXboxController driverController;

  /** Previous requested chassis speed. */
  private ChassisSpeeds previousChassisSpeeds;

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

  public DriveCommand(CommandXboxController driverController) {
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance();

    addRequirements(swerve);

    previousChassisSpeeds = new ChassisSpeeds();

    this.driverController = driverController;
    previousRequest = DriveRequest.fromController(driverController);

    headingEntry = Telemetry.addDoubleEntry("swerve/driveCommand", "heading");
    headingVelocityEntry = Telemetry.addDoubleEntry("swerve/driveCommand", "headingVelocity");

    headingGoalEntry = Telemetry.addDoubleEntry("swerve/driveCommand", "headingGoal");

    headingSetpointEntry = Telemetry.addDoubleEntry("swerve/driveCommand", "headingSetpoint");
    headingVelocitySetpointEntry =
        Telemetry.addDoubleEntry("swerve/driveCommand", "headingVelocitySetpoint");
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    request = DriveRequest.fromController(driverController);

    Translation2d translationVelocityMetersPerSecond =
        request.translationVector.times(SwerveConstants.MAXIMUM_SPEED);

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

    chassisSpeeds.vxMetersPerSecond =
        MathUtil.clamp(
            chassisSpeeds.vxMetersPerSecond,
            previousChassisSpeeds.vxMetersPerSecond
                - SwerveConstants.MAXIMUM_ACCELERATION * RobotConstants.PERIODIC_DURATION,
            previousChassisSpeeds.vxMetersPerSecond
                + SwerveConstants.MAXIMUM_ACCELERATION * RobotConstants.PERIODIC_DURATION);

    chassisSpeeds.vyMetersPerSecond =
        MathUtil.clamp(
            chassisSpeeds.vyMetersPerSecond,
            previousChassisSpeeds.vyMetersPerSecond
                - SwerveConstants.MAXIMUM_ACCELERATION * RobotConstants.PERIODIC_DURATION,
            previousChassisSpeeds.vyMetersPerSecond
                + SwerveConstants.MAXIMUM_ACCELERATION * RobotConstants.PERIODIC_DURATION);

    double maxOmegaRadiansPerSecond =
        Units.rotationsToRadians(SwerveConstants.MAXIMUM_ATTAINABLE_ROTATION_SPEED);

    chassisSpeeds.omegaRadiansPerSecond =
        MathUtil.clamp(
            chassisSpeeds.omegaRadiansPerSecond,
            -maxOmegaRadiansPerSecond,
            maxOmegaRadiansPerSecond);

    headingVelocityEntry.set(Units.radiansToRotations(chassisSpeeds.omegaRadiansPerSecond));

    swerve.setChassisSpeeds(chassisSpeeds);

    previousChassisSpeeds = chassisSpeeds;

    previousRequest = request;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
