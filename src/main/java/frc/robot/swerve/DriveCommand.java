package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.odometry.Odometry;

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

  /* Heading feedback controller. */
  private final PIDController headingFeedback;

  /** Heading goal. */
  private State headingGoal, headingSetpoint;

  public DriveCommand(CommandXboxController driverController) {
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance();

    addRequirements(swerve);

    headingFeedback = new PIDController(6.0, 0.0, 0.1);
    headingFeedback.enableContinuousInput(-0.5, 0.5);

    headingGoal = new State(0.0, 0.0);
    headingSetpoint = new State(0.0, 0.0);

    previousChassisSpeeds = new ChassisSpeeds();

    this.driverController = driverController;
    previousRequest = DriveRequest.fromController(driverController);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    request = DriveRequest.fromController(driverController);

    Translation2d translationVelocityMetersPerSecond =
        request.translation().times(SwerveConstants.MAXIMUM_SPEED);

    Rotation2d poseHeading = odometry.getPosition().getRotation();

    if (DriveRequest.startedDrifting(previousRequest, request)) {
      headingGoal = wrap(new State(poseHeading.getRotations(), 0.0), poseHeading.getRotations());
    }

    if (request.isSnapping()) {
      headingGoal =
          wrap(new State(request.heading().getRotations(), 0.0), poseHeading.getRotations());
    }

    double omegaRotationsPerSecond = 0.0;

    if (request.isSpinning()) {
      headingSetpoint =
          wrap(
              new State(poseHeading.getRotations(), request.omega().getRotations()),
              poseHeading.getRotations());

      omegaRotationsPerSecond = headingSetpoint.velocity;
    } else {
      headingSetpoint =
          SwerveConstants.ROTATION_MOTION_PROFILE.calculate(
              RobotConstants.PERIODIC_DURATION, headingSetpoint, headingGoal);

      omegaRotationsPerSecond =
          headingFeedback.calculate(poseHeading.getRotations(), headingSetpoint.position);
    }

    ChassisSpeeds chassisSpeeds;

    if (request.isRobotCentric()) {
      chassisSpeeds =
          new ChassisSpeeds(
              translationVelocityMetersPerSecond.getX(),
              translationVelocityMetersPerSecond.getY(),
              Units.rotationsToRadians(omegaRotationsPerSecond));
    } else {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationVelocityMetersPerSecond.getX(),
              translationVelocityMetersPerSecond.getY(),
              Units.rotationsToRadians(omegaRotationsPerSecond),
              poseHeading);
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

    chassisSpeeds.omegaRadiansPerSecond =
        MathUtil.clamp(
            chassisSpeeds.omegaRadiansPerSecond,
            -SwerveConstants.MAXIMUM_ATTAINABLE_ROTATION_SPEED.getRadians(),
            SwerveConstants.MAXIMUM_ATTAINABLE_ROTATION_SPEED.getRadians());

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

  /**
   * Wraps an input state to be within the range of a measured position.
   *
   * @param state the input state.
   * @param measurement the measured position.
   * @return a new state within the range of the measured position.
   */
  private State wrap(State state, double measurement) {
    double minError = MathUtil.inputModulus(state.position - measurement, -0.5, 0.5);
    return new State(minError + measurement, state.velocity);
  }
}
