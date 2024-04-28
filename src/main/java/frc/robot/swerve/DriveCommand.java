package frc.robot.swerve;

import java.util.function.Function;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.odometry.Odometry;

/** Drives the swerve using driver input. */
public class DriveCommand extends Command {
  /** Swerve subsystem. */
  private final Swerve swerve;

  /** Odometry subsystem. */
  private final Odometry odometry;

  /** Controller used to get driver input. */
  private final CommandXboxController driverController;

  /** Translation acceleration limiter. */
  private final SlewRateLimiter xAccelerationLimiter;
  
  /** Translation acceleration limiter. */
  private final SlewRateLimiter yAccelerationLimiter;

  /** Rotation velocity clamper. */
  private final Function<Double, Double> rotationVelocityClamper;

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

    xAccelerationLimiter = swerve.translationMotionProfileConfig().createAccelerationLimiter();
    yAccelerationLimiter = swerve.translationMotionProfileConfig().createAccelerationLimiter();

    rotationVelocityClamper = swerve.rotationMotionProfileConfig().createVelocityClamper();

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
                request.translationAxis().getX() * swerve.maximumTranslationVelocity(),
                request.translationAxis().getY() * swerve.maximumTranslationVelocity(),
                swerve.maximumRotationVelocity().times(request.rotationVelocityAxis()).getRadians(),
                odometry.getDriverRelativeHeading()));

    swerve.setChassisSpeeds(chassisSpeeds);
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
    double vxMetersPerSecond = xAccelerationLimiter.calculate(desiredChassisSpeeds.vxMetersPerSecond);
    double vyMetersPerSecond = yAccelerationLimiter.calculate(desiredChassisSpeeds.vyMetersPerSecond);
    double omegaRadiansPerSecond = rotationVelocityClamper.apply(Units.radiansToRotations(desiredChassisSpeeds.omegaRadiansPerSecond));

    return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
  }
}
