package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Drive request. */
public record DriveRequest(
    DriveRequest.TranslationMode translationMode,
    DriveRequest.RotationMode rotationMode,
    Translation2d translationAxis,
    Translation2d rotationAxis) {

  /** Translation mode. */
  private enum TranslationMode {
    /** Field-centric driving. */
    FIELD_CENTRIC,
    /** Robot-centric driving. */
    ROBOT_CENTRIC
  }

  /** Rotation mode. */
  private enum RotationMode {
    /** Drifting (no rotation requested). */
    DRIFTING,
    /** Spinning (velocity requested). */
    SPINNING,
    /** Aligning (heading requested). */
    ALIGNING
  }

  /**
   * Returns true if no rotation is requested.
   *
   * @param heading the heading axis.
   * @param aligning true if aligning is requested.
   * @return
   */
  private static boolean isDrifting(Translation2d heading, boolean aligning) {
    if (aligning) {
      return heading.getNorm() < 0.7;
    }

    return Math.abs(heading.getY()) < 0.1;
  }

  /**
   * Creates a new driver request from controller inputs.
   *
   * @param controller the input controller.
   * @return a new driver request from controller inputs.
   */
  public static DriveRequest fromController(CommandXboxController controller) {
    boolean snipingRequested = Math.abs(controller.getLeftTriggerAxis()) > 0.5;
    boolean aligningRequested = Math.abs(controller.getRightTriggerAxis()) > 0.5;

    double translationX = -controller.getLeftY();

    double translationY = -controller.getLeftX();

    double translationMagnitude = Math.hypot(translationX, translationY);
    Rotation2d translationDirection = new Rotation2d(translationX, translationY);

    translationMagnitude = MathUtil.applyDeadband(translationMagnitude, 0.1);
    translationMagnitude =
        Math.copySign(translationMagnitude * translationMagnitude, translationMagnitude);

    if (snipingRequested) {
      translationMagnitude *= 0.25;
    }

    Translation2d translationVector = new Translation2d(translationMagnitude, translationDirection);

    TranslationMode translationMode = TranslationMode.FIELD_CENTRIC;

    Translation2d rotationVector =
        new Translation2d(-controller.getRightY(), -controller.getRightX());

    RotationMode rotationMode;

    if (isDrifting(rotationVector, aligningRequested)) {
      rotationMode = RotationMode.DRIFTING;
    } else if (aligningRequested) {
      rotationMode = RotationMode.ALIGNING;
    } else {
      rotationMode = RotationMode.SPINNING;
    }

    return new DriveRequest(translationMode, rotationMode, translationVector, rotationVector);
  }

  /**
   * Returns the requested translation velocity in meters per second.
   *
   * @return the requested translation velocity in meters per second.
   */
  public Translation2d velocity() {
    return translationAxis.times(SwerveConstants.MAXIMUM_SPEED);
  }

  /**
   * Returns the requested rotation velocity.
   *
   * @return the requested rotation velocity.
   */
  public Rotation2d omega() {
    if (Math.abs(rotationAxis.getY()) < 0.1) return new Rotation2d();

    return SwerveConstants.MAXIMUM_ROTATION_SPEED.times(rotationAxis.getY());
  }
}
