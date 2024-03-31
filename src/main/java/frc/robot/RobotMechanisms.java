package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.InterpolatableColor;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;
import frc.robot.intake.IntakeConstants.RollerMotorConstants;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;
import frc.robot.superstructure.SuperstructureState;

/** Helper class for rendering robot mechanisms. */
public class RobotMechanisms {

  private static RobotMechanisms instance = null;

  private final Mechanism2d mechanism;

  private MechanismLigament2d shoulder, flywheel, serializer, pivot;

  private final double WIDTH =
      2
          * (RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE
              + RobotConstants.MAX_HORIZONTAL_EXTENSION_DISTANCE);

  private final double HEIGHT = RobotConstants.MAX_VERTICAL_EXTENSION_DISTANCE;

  private final Translation2d ORIGIN = new Translation2d(WIDTH / 2, 0);

  public static final Translation2d SHOULDER_TO_ORIGIN =
      new Translation2d(Units.inchesToMeters(-11.361), Units.inchesToMeters(7.721));

  private final Color8Bit DEFAULT_COLOR = new Color8Bit(Color.kLightGray);

  private final InterpolatableColor flywheelColor =
      new InterpolatableColor(Color.kLightGray, Color.kSalmon);
  private final InterpolatableColor serializerColor =
      new InterpolatableColor(Color.kLightGray, Color.kCornflowerBlue);
  private final InterpolatableColor intakeColor =
      new InterpolatableColor(Color.kLightGray, Color.kPaleGreen);

  private RobotMechanisms() {
    mechanism = new Mechanism2d(WIDTH, HEIGHT);

    initializeArmMechanism();
    initializeIntakeMechanism();

    initializeFramePerimeterMechanisms();
  }

  private void initializeArmMechanism() {
    Translation2d armRootTranslation = ORIGIN.plus(SHOULDER_TO_ORIGIN);

    double armThickness = Units.inchesToMeters(2) * 100;

    MechanismRoot2d armRoot =
        mechanism.getRoot("arm", armRootTranslation.getX(), armRootTranslation.getY());

    shoulder =
        armRoot.append(
            new MechanismLigament2d(
                "shoulder",
                ShoulderMotorConstants.JOINT_CONSTANTS.lengthMeters(),
                0,
                armThickness,
                DEFAULT_COLOR));
    flywheel =
        shoulder.append(
            new MechanismLigament2d(
                "wrist", Units.inchesToMeters(4.321), 0, armThickness, DEFAULT_COLOR));

    serializer =
        shoulder.append(
            new MechanismLigament2d(
                "serializer", Units.inchesToMeters(8.771), 0, armThickness, DEFAULT_COLOR));
  }

  private void initializeFramePerimeterMechanisms() {
    double framePerimeterThickness = Units.inchesToMeters(1) * 10;

    mechanism
        .getRoot(
            "framePerimeterLeft",
            ORIGIN.getX() - RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE,
            0)
        .append(
            new MechanismLigament2d(
                "framePerimeterLeft_", HEIGHT, 90, framePerimeterThickness, DEFAULT_COLOR));

    mechanism
        .getRoot(
            "framePerimeterRight",
            ORIGIN.getX() + RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE,
            0)
        .append(
            new MechanismLigament2d(
                "framePerimeterRight_", HEIGHT, 90, framePerimeterThickness, DEFAULT_COLOR));
  }

  private void initializeIntakeMechanism() {
    double intakeThickness = Units.inchesToMeters(3) * 100;

    pivot =
        mechanism
            .getRoot(
                "intake",
                ORIGIN.getX() + Units.inchesToMeters(13.164),
                ORIGIN.getY() + Units.inchesToMeters(6.283))
            .append(
                new MechanismLigament2d(
                    "intake_", PivotMotorConstants.DISTANCE, 0.0, intakeThickness, DEFAULT_COLOR));
  }

  public static RobotMechanisms getInstance() {
    if (instance == null) {
      instance = new RobotMechanisms();
    }

    return instance;
  }

  public Mechanism2d getMechanism() {
    return mechanism;
  }

  public void updateSuperstructure(SuperstructureState state) {
    Rotation2d shoulderRotation = Rotation2d.fromRotations(state.shoulderAngleRotations().position);
    Rotation2d wristRotation = Rotation2d.fromRotations(state.wristAngleRotations().position);

    shoulder.setAngle(shoulderRotation);

    // Offset the rendered wrist rotation so that zero degrees is perpendicular to the shoulder,
    // rather than parallel with the shoulder
    Rotation2d offsetWristRotation = wristRotation.plus(Rotation2d.fromDegrees(90));
    Rotation2d shooterRotation = offsetWristRotation;
    Rotation2d serializerRotation = offsetWristRotation.plus(Rotation2d.fromDegrees(180));

    pivot.setAngle(Rotation2d.fromRotations(state.pivotAngleRotations().position));
    pivot.setColor(
        new Color8Bit(
            intakeColor.sample(
                Math.abs(state.rollerVelocityRotationsPerSecond()),
                0,
                RollerMotorConstants.MAXIMUM_SPEED)));

    flywheel.setAngle(shooterRotation);
    flywheel.setColor(
        new Color8Bit(
            flywheelColor.sample(
                Math.abs(state.flywheelVelocityRotationsPerSecond()),
                0,
                FlywheelConstants.MAXIMUM_TANGENTIAL_SPEED)));

    serializer.setAngle(serializerRotation);
    serializer.setColor(
        new Color8Bit(
            serializerColor.sample(
                Math.abs(state.serializerVelocityRotationsPerSecond()),
                0,
                SerializerConstants.MAXIMUM_TANGENTIAL_SPEED)));
  }
}
