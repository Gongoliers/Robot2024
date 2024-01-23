package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.CustomXboxController;

public class DriveRequest {

  public enum TranslationMode {
    FIELD_CENTRIC,
    ROBOT_CENTRIC
  }

  public enum RotationMode {
    DRIFTING,
    SPINNING,
    SNAPPING
  }

  public final TranslationMode translationMode;
  public final RotationMode rotationMode;

  public final Translation2d translationVector;
  public final Translation2d rotationVector;

  private DriveRequest(
      TranslationMode translationMode,
      RotationMode rotationMode,
      Translation2d translationVector,
      Translation2d rotationVector) {
    this.translationMode = translationMode;
    this.rotationMode = rotationMode;

    this.translationVector = translationVector;
    this.rotationVector = rotationVector;
  }

  private static boolean isDrifting(Translation2d heading, boolean aligning) {
    if (aligning) {
      final double kMinHeadingDisplacement = 0.7;

      return heading.getNorm() < kMinHeadingDisplacement;
    }

    final double kOmegaDeadband = 0.1;

    return Math.abs(heading.getY()) < kOmegaDeadband;
  }

  static DriveRequest fromController(CustomXboxController controller) {
    boolean snipingRequested = controller.leftTrigger().getAsBoolean();
    boolean aligningRequested = controller.rightTrigger().getAsBoolean();

    Translation2d translationVector =
        new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    Translation2d rotationVector =
        new Translation2d(-controller.getRightY(), -controller.getRightX());

    TranslationMode translationMode =
        snipingRequested ? TranslationMode.ROBOT_CENTRIC : TranslationMode.FIELD_CENTRIC;

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

  public Translation2d getTranslationVelocity() {
    double scalar = SwerveConstants.MAXIMUM_SPEED;

    if (translationMode == TranslationMode.ROBOT_CENTRIC) {
      scalar *= 0.25;
    }

    return translationVector.times(scalar);
  }

  public double getRotationVelocity() {
    return SwerveConstants.MAXIMUM_ROTATION_SPEED * this.rotationVector.getY();
  }

  private Rotation2d snapToNearest(Rotation2d angle, Rotation2d multiple) {
    double snappedRadians = snapToNearest(angle.getRadians(), multiple.getRadians());

    return Rotation2d.fromRadians(snappedRadians);
  }

  private double snapToNearest(double n, double multiple) {
    return Math.round(n / multiple) * multiple;
  }

  public Rotation2d getHeading() {
    double kSnapMultipleDegrees = 90;

    return snapToNearest(rotationVector.getAngle(), Rotation2d.fromDegrees(kSnapMultipleDegrees));
  }
}
