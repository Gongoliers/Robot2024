package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Superstructure mechanism renderer. */
public class SuperstructureMechanism {

  /** Superstructure mechanism renderer singleton. */
  private static SuperstructureMechanism instance = null;

  /** Superstructure mechanism. */
  private final Mechanism2d mechanism;

  /** Superstructure mechanism ligaments. */
  private MechanismLigament2d shoulder, flywheel, serializer, rollers;

  /** Distance from the frame perimeter to the origin in meters. */
  public static final double FRAME_PERIMETER_TO_ORIGIN_DISTANCE = Units.inchesToMeters(14);

  /** Maximum horizontal extension distance in meters. */
  public static final double MAX_HORIZONTAL_EXTENSION_DISTANCE = Units.feetToMeters(1);

  /** Maximum vertical extension distance in meters. */
  public static final double MAX_VERTICAL_EXTENSION_DISTANCE = Units.inchesToMeters(48);

  /** Superstructure mechanism width. */
  private final double WIDTH =
      2 * (FRAME_PERIMETER_TO_ORIGIN_DISTANCE + MAX_HORIZONTAL_EXTENSION_DISTANCE);

  /** Superstructure mechanism height. */
  private final double HEIGHT = MAX_VERTICAL_EXTENSION_DISTANCE;

  /** Superstructure mechanism origin (horizontally centered). */
  private final Translation2d ORIGIN = new Translation2d(WIDTH / 2, 0);

  /** Translation between the origin to shoulder base. */
  private final Translation2d ORIGIN_TO_SHOULDER_BASE =
      new Translation2d(Units.inchesToMeters(-7.5), Units.inchesToMeters(3.75));

  /** Height of the shoulder member. */
  private final double SHOULDER_BASE_HEIGHT = Units.inchesToMeters(18);

  /** Distance from top of the shoulder member to shoulder pivot. */
  private final double SHOULDER_BASE_TOP_TO_SHOULDER = Units.inchesToMeters(1.5);

  /** Translation from shoulder base to shoulder pivot. */
  private final Translation2d SHOULDER_BASE_TO_SHOULDER =
      new Translation2d(0, SHOULDER_BASE_HEIGHT - SHOULDER_BASE_TOP_TO_SHOULDER);

  /** Length of the flywheel segment of the shooter. */
  private final double SHOOTER_LENGTH = Units.inchesToMeters(12);

  /** Length of the serializer segment of the shooter. */
  private final double SERIALIZER_LENGTH = Units.inchesToMeters(7.5);

  /** Translation from origin to the intake rollers. */
  private final Translation2d ORIGIN_TO_ROLLERS =
      new Translation2d(Units.inchesToMeters(5.75), Units.inchesToMeters(2.5));

  /** Superstructure mechanism default color. */
  private final Color8Bit DEFAULT_COLOR = new Color8Bit(Color.kLightGray);

  /** Superstructure mechanism flywheel color. */
  private final Color8Bit FLYWHEEL_COLOR = new Color8Bit(Color.kSalmon);

  /** Superstructure mechanism serializer color. */
  private final Color8Bit SERIALIZER_COLOR = new Color8Bit(Color.kCornflowerBlue);

  /** Superstructure mechanism rollers color. */
  private final Color8Bit ROLLERS_COLOR = new Color8Bit(Color.kSpringGreen);

  /** Initializes superstructure mechanism renderer. */
  private SuperstructureMechanism() {
    mechanism = new Mechanism2d(WIDTH, HEIGHT);

    Translation2d armRootTranslation = ORIGIN.plus(ORIGIN_TO_SHOULDER_BASE);

    double armThickness = Units.inchesToMeters(2) * 100;

    MechanismRoot2d armRoot =
        mechanism.getRoot("arm", armRootTranslation.getX(), armRootTranslation.getY());

    MechanismLigament2d shoulderBase =
        armRoot.append(
            new MechanismLigament2d(
                "base", SHOULDER_BASE_TO_SHOULDER.getY(), 90, armThickness, DEFAULT_COLOR));

    shoulder =
        shoulderBase.append(
            new MechanismLigament2d(
                "shoulder", Units.inchesToMeters(16.775), 0, armThickness, DEFAULT_COLOR));

    flywheel =
        shoulder.append(
            new MechanismLigament2d("shooter", SHOOTER_LENGTH, 145, armThickness, DEFAULT_COLOR));

    serializer =
        shoulder.append(
            new MechanismLigament2d(
                "serializer", SERIALIZER_LENGTH, -35, armThickness, DEFAULT_COLOR));

    Translation2d rollersRootTranslation = ORIGIN.plus(ORIGIN_TO_ROLLERS);

    rollers =
        mechanism
            .getRoot("intake", rollersRootTranslation.getX(), rollersRootTranslation.getY())
            .append(
                new MechanismLigament2d(
                    "rollers", Units.inchesToMeters(12), 0, armThickness, DEFAULT_COLOR));

    double framePerimeterThickness = Units.inchesToMeters(1) * 10;

    mechanism
        .getRoot("framePerimeterLeft", ORIGIN.getX() - FRAME_PERIMETER_TO_ORIGIN_DISTANCE, 0)
        .append(
            new MechanismLigament2d(
                "framePerimeterLeft_", HEIGHT, 90, framePerimeterThickness, DEFAULT_COLOR));

    mechanism
        .getRoot("framePerimeterRight", ORIGIN.getX() + FRAME_PERIMETER_TO_ORIGIN_DISTANCE, 0)
        .append(
            new MechanismLigament2d(
                "framePerimeterRight_", HEIGHT, 90, framePerimeterThickness, DEFAULT_COLOR));
  }

  /**
   * Returns the superstructure mechanism renderer instance.
   *
   * @return the superstructure mechanism renderer instance.
   */
  public static SuperstructureMechanism getInstance() {
    if (instance == null) {
      instance = new SuperstructureMechanism();
    }

    return instance;
  }

  /**
   * Returns the superstructure mechanism renderer.
   *
   * @return the superstructure mechanism renderer.
   */
  public Mechanism2d getMechanism() {
    return mechanism;
  }

  /**
   * Updates the superstructure mechanism renderer.
   *
   * @param state the state to render.
   */
  public void update(SuperstructureState state) {
    Rotation2d shoulderRotation =
        Rotation2d.fromRotations(state.armState().shoulderRotations().position);

    Rotation2d offsetShoulderRotation = shoulderRotation.minus(Rotation2d.fromDegrees(90));

    shoulder.setAngle(offsetShoulderRotation);

    if (state.shooterState().flywheelVelocityRotationsPerSecond() != 0.0) {
      flywheel.setColor(FLYWHEEL_COLOR);
    } else {
      flywheel.setColor(DEFAULT_COLOR);
    }

    if (state.shooterState().serializerVelocityRotationsPerSecond() != 0.0) {
      serializer.setColor(SERIALIZER_COLOR);
    } else {
      serializer.setColor(DEFAULT_COLOR);
    }

    if (state.intakeState().frontRollerVelocityRotationsPerSecond() != 0.0
        || state.intakeState().backRollerVelocityRotationsPerSecond() != 0) {
      rollers.setColor(ROLLERS_COLOR);
    } else {
      rollers.setColor(DEFAULT_COLOR);
    }
  }
}
