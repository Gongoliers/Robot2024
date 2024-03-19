package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.SnapRotation;
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

  /** Heading snapper. */
  private final SnapRotation headingSnapper;

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

    headingSnapper = SnapRotation.to(Rotation2d.fromDegrees(90));

    odometry.onYawUpdate(newPose -> resetHeadingGoal());
  }

  @Override
  public void initialize() {
    resetHeadingGoal();
  }

  @Override
  public void execute() {
    request = DriveRequest.fromController(driverController);

    Translation2d translationVelocityMetersPerSecond =
        request.translation().times(SwerveConstants.MAXIMUM_SPEED);

    Rotation2d driverRelativeHeading = odometry.getDriverRelativeHeading();

    if (DriveRequest.startedDrifting(previousRequest, request)) {
      resetHeadingGoal();
    }

    if (request.isSnapping()) {
      setPositionHeadingGoal(headingSnapper.snap(request.driverHeading()));
    }

    Rotation2d omega;

    if (request.isSpinning()) {
      updateVelocity(request.omega());

      omega = request.omega();
    } else {
      omega = calculateHeadingProfileOmega();
    }

    ChassisSpeeds chassisSpeeds =
        clampChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationVelocityMetersPerSecond.getX(),
                translationVelocityMetersPerSecond.getY(),
                omega.getRadians(),
                driverRelativeHeading));

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
   * Clamps desired chassis speeds to be within velocity and acceleration constraints.
   *
   * @param desiredChassisSpeeds the desired chassis speeds.
   * @return the clamped chassis speeds.
   */
  private ChassisSpeeds clampChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
    double vxMetersPerSecond =
        MathUtil.clamp(
            desiredChassisSpeeds.vxMetersPerSecond,
            previousChassisSpeeds.vxMetersPerSecond
                - SwerveConstants.MAXIMUM_ACCELERATION * RobotConstants.PERIODIC_DURATION,
            previousChassisSpeeds.vxMetersPerSecond
                + SwerveConstants.MAXIMUM_ACCELERATION * RobotConstants.PERIODIC_DURATION);

    double vyMetersPerSecond =
        MathUtil.clamp(
            desiredChassisSpeeds.vyMetersPerSecond,
            previousChassisSpeeds.vyMetersPerSecond
                - SwerveConstants.MAXIMUM_ACCELERATION * RobotConstants.PERIODIC_DURATION,
            previousChassisSpeeds.vyMetersPerSecond
                + SwerveConstants.MAXIMUM_ACCELERATION * RobotConstants.PERIODIC_DURATION);

    double omegaRadiansPerSecond =
        MathUtil.clamp(
            desiredChassisSpeeds.omegaRadiansPerSecond,
            -SwerveConstants.MAXIMUM_ATTAINABLE_ROTATION_SPEED.getRadians(),
            SwerveConstants.MAXIMUM_ATTAINABLE_ROTATION_SPEED.getRadians());

    return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
  }

  /**
   * Returns the reference heading to use with the heading motion profile.
   *
   * @return the reference heading to use with the heading motion profile.
   */
  private Rotation2d getReferenceHeading() {
    return odometry.getDriverRelativeHeading();
  }

  /**
   * Resets the heading goal.
   *
   * <p>The robot's orientation becomes the new goal and the motion profile is reset by setting the
   * setpoint to the goal.
   */
  private void resetHeadingGoal() {
    setPositionHeadingGoal(getReferenceHeading());

    headingSetpoint = headingGoal;
  }

  /**
   * Sets the position goal.
   *
   * @param goal the position goal.
   */
  private void setPositionHeadingGoal(Rotation2d goal) {
    State state = new State(goal.getRotations(), 0.0);

    headingGoal = wrapState(state, getReferenceHeading().getRotations());
  }

  /**
   * Updates the motion profile with the robot's rotational velocity.
   *
   * @param omega the robot's rotational velocity.
   */
  private void updateVelocity(Rotation2d omega) {
    Rotation2d heading = getReferenceHeading();

    State state = new State(heading.getRotations(), omega.getRotations());

    headingGoal = wrapState(state, heading.getRotations());
  }

  /**
   * Calculates the robot's rotational velocity.
   *
   * @return the robot's calculated rotational velocity.
   */
  private Rotation2d calculateHeadingProfileOmega() {
    headingSetpoint =
        SwerveConstants.ROTATION_MOTION_PROFILE.calculate(
            RobotConstants.PERIODIC_DURATION, headingSetpoint, headingGoal);

    double omegaRotationsPerSecond =
        headingFeedback.calculate(getReferenceHeading().getRotations(), headingSetpoint.position);

    return Rotation2d.fromRotations(omegaRotationsPerSecond);
  }

  /**
   * Wraps an input state to be within the range of a measured position.
   *
   * @param state the input state.
   * @param measurement the measured position.
   * @return a new state within the range of the measured position.
   */
  private State wrapState(State state, double measurement) {
    double minError = MathUtil.inputModulus(state.position - measurement, -0.5, 0.5);
    return new State(minError + measurement, state.velocity);
  }
}
