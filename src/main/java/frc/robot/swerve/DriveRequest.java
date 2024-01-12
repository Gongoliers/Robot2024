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

  public final Translation2d velocity;
  public final Translation2d heading;

  private DriveRequest(
      TranslationMode translation,
      RotationMode rotation,
      Translation2d velocity,
      Translation2d heading) {
    this.translationMode = translation;
    this.rotationMode = rotation;

    this.velocity = velocity;
    this.heading = heading;
  }

  static DriveRequest fromController(CustomXboxController controller) {
    final boolean snipingRequested = controller.leftTrigger().getAsBoolean();
    final boolean aligningRequested = controller.rightTrigger().getAsBoolean();

    final Translation2d translation =
        new Translation2d(-controller.getLeftY(), -controller.getLeftX());
    final Translation2d heading =
        new Translation2d(-controller.getRightY(), -controller.getRightX());

    final TranslationMode translationMode =
        snipingRequested ? TranslationMode.ROBOT_CENTRIC : TranslationMode.FIELD_CENTRIC;

    final boolean isDrifting = determineIfDrifting(heading, aligningRequested);

    final RotationMode rotationMode;

    if (isDrifting) {
      rotationMode = RotationMode.DRIFTING;
    } else if (aligningRequested) {
      rotationMode = RotationMode.SNAPPING;
    } else {
      rotationMode = RotationMode.SPINNING;
    }

    return new DriveRequest(translationMode, rotationMode, translation, heading);
  }

  private static boolean determineIfDrifting(Translation2d heading, boolean aligning) {
    if (aligning) {
      final double kMinHeadingDisplacement = 0.7;

      return heading.getNorm() < kMinHeadingDisplacement;
    }

    final double kOmegaDeadband = 0.1;

    return Math.abs(heading.getY()) < kOmegaDeadband;
  }

  public static boolean startedDrifting(DriveRequest past, DriveRequest present) {
    return past.rotationMode == RotationMode.SPINNING
        && present.rotationMode == RotationMode.DRIFTING;
  }

  public Translation2d getRequestedVelocity() {
    double scalar = SwerveConstants.MAXIMUM_SPEED;

    if (translationMode == TranslationMode.ROBOT_CENTRIC) {
      scalar *= 0.25;
    }

    return velocity.times(scalar);
  }

  public Rotation2d getRequestedSpinRate() {
    return Rotation2d.fromRotations(0.5).times(this.heading.getY());
  }

  private Rotation2d snapToNearest(Rotation2d angle, Rotation2d multiple) {
    double snappedRadians = snapToNearest(angle.getRadians(), multiple.getRadians());

    return Rotation2d.fromRadians(snappedRadians);
  }

  private double snapToNearest(double n, double multiple) {
    return Math.round(n / multiple) * multiple;
  }

  public Rotation2d getRequestedSnapAngle() {
    double kSnapMultipleDegrees = 90;

    return snapToNearest(heading.getAngle(), Rotation2d.fromDegrees(kSnapMultipleDegrees));
  }
}
