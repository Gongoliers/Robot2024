package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants;
import frc.robot.odometry.Odometry;

/** Drives the swerve using driver input. */
public class DriveCommand extends Command {
  /** Swerve subsystem. */
  private final Swerve swerve;

  /** Odometry subsystem. */
  private final Odometry odometry;

  /** Controller used to get driver input. */
  private final CommandXboxController driverController;

  /** Previous requested chassis speed. */
  private ChassisSpeeds previousChassisSpeeds;

  /** Request from the driver controller. */
  private DriveRequest request;

  /**
   * Initializes the drive command.
   *
   * @param driverController controller to use as input.
   */
  public DriveCommand(CommandXboxController driverController) {
    swerve = Swerve.getInstance();
    odometry = Odometry.getInstance();

    this.driverController = driverController;

    addRequirements(swerve);

    previousChassisSpeeds = new ChassisSpeeds();

    request = DriveRequest.fromController(driverController);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    request = DriveRequest.fromController(driverController);

    final ChassisSpeeds chassisSpeeds =
        clampChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                request.velocity().getX(),
                request.velocity().getY(),
                request.omega().getRadians(),
                odometry.getDriverRelativeHeading()));

    swerve.setChassisSpeeds(chassisSpeeds);

    previousChassisSpeeds = chassisSpeeds;
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
            -SwerveConstants.MAXIMUM_ROTATION_SPEED.getRadians(),
            SwerveConstants.MAXIMUM_ROTATION_SPEED.getRadians());

    return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
  }
}
