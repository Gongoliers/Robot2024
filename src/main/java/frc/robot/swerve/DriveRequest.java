package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public record DriveRequest(
    DriveRequest.TranslationMode translationMode,
    DriveRequest.RotationMode rotationMode,
    Translation2d translation,
    Translation2d rotation) {

  public enum TranslationMode {
    FIELD_CENTRIC,
    ROBOT_CENTRIC
  }

  public enum RotationMode {
    DRIFTING,
    SPINNING,
    SNAPPING
  }

  private static boolean isDrifting(Translation2d heading, boolean aligning) {
    if (aligning) {
      final double kMinHeadingDisplacement = 0.7;

      return heading.getNorm() < kMinHeadingDisplacement;
    }

    final double kOmegaDeadband = 0.1;

    return Math.abs(heading.getY()) < kOmegaDeadband;
  }

  public static DriveRequest fromController(CommandXboxController controller) {
    boolean snipingRequested = controller.leftTrigger().getAsBoolean();
    boolean aligningRequested = controller.rightTrigger().getAsBoolean();

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

    TranslationMode translationMode =
        snipingRequested ? TranslationMode.ROBOT_CENTRIC : TranslationMode.FIELD_CENTRIC;

    Translation2d rotationVector =
        new Translation2d(-controller.getRightY(), -controller.getRightX());

    RotationMode rotationMode;

    if (isDrifting(rotationVector, aligningRequested)) {
      rotationMode = RotationMode.DRIFTING;
    } else if (aligningRequested) {
      rotationMode = RotationMode.SNAPPING;
    } else {
      rotationMode = RotationMode.SPINNING;
    }

    return new DriveRequest(translationMode, rotationMode, translationVector, rotationVector);
  }

  public static boolean startedDrifting(DriveRequest past, DriveRequest present) {
    return past.rotationMode == RotationMode.SPINNING
        && present.rotationMode == RotationMode.DRIFTING;
  }

  public boolean isRobotCentric() {
    return translationMode == TranslationMode.ROBOT_CENTRIC;
  }

  public boolean isSpinning() {
    return rotationMode == RotationMode.SPINNING;
  }

  public boolean isSnapping() {
    return rotationMode == RotationMode.SNAPPING;
  }

  public boolean isDrifting() {
    return rotationMode == RotationMode.DRIFTING;
  }

  public Rotation2d omega() {
    if (Math.abs(this.rotation().getY()) < 0.1) return new Rotation2d();

    return SwerveConstants.MAXIMUM_ROTATION_SPEED.times(this.rotation().getY() * 0.9);
  }

  public Rotation2d driverHeading() {
    return rotation().getAngle();
  }
}
