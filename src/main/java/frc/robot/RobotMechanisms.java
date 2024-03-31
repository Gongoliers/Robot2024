package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;
import frc.robot.superstructure.SuperstructureState;

/** Helper class for rendering robot mechanisms. */
public class RobotMechanisms {

  private static RobotMechanisms instance = null;

  private final Mechanism2d mechanism;

  private MechanismLigament2d shoulder;

  private final double WIDTH =
      2
          * (RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE
              + RobotConstants.MAX_HORIZONTAL_EXTENSION_DISTANCE);

  private final double HEIGHT = RobotConstants.MAX_VERTICAL_EXTENSION_DISTANCE;

  private final Translation2d ORIGIN = new Translation2d(WIDTH / 2, 0);

  private final Translation2d ORIGIN_TO_SHOULDER_BASE = new Translation2d(Units.inchesToMeters(-7.5), Units.inchesToMeters(3.75));

  private final double SHOULDER_BASE_HEIGHT = Units.inchesToMeters(18);

  private final double SHOULDER_BASE_TOP_TO_SHOULDER = Units.inchesToMeters(1.5);

  private final Translation2d SHOULDER_BASE_TO_SHOULDER = new Translation2d(0, SHOULDER_BASE_HEIGHT - SHOULDER_BASE_TOP_TO_SHOULDER);

  private final Color8Bit DEFAULT_COLOR = new Color8Bit(Color.kLightGray);

  private RobotMechanisms() {
    mechanism = new Mechanism2d(WIDTH, HEIGHT);

    initializeArmMechanism();

    initializeFramePerimeterMechanisms();
  }

  private void initializeArmMechanism() {
    Translation2d armRootTranslation = ORIGIN.plus(ORIGIN_TO_SHOULDER_BASE);

    double armThickness = Units.inchesToMeters(2) * 100;

    MechanismRoot2d armRoot =
        mechanism.getRoot("arm", armRootTranslation.getX(), armRootTranslation.getY());

    MechanismLigament2d shoulderBase = armRoot.append(new MechanismLigament2d("base", SHOULDER_BASE_TO_SHOULDER.getY(), 90, armThickness, DEFAULT_COLOR));

    shoulder =
        shoulderBase.append(
            new MechanismLigament2d(
                "shoulder",
                ShoulderMotorConstants.JOINT_CONSTANTS.lengthMeters(),
                0,
                armThickness,
                DEFAULT_COLOR));
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

    Rotation2d offsetShoulderRotation = shoulderRotation.minus(Rotation2d.fromDegrees(90));

    shoulder.setAngle(offsetShoulderRotation);
  }
}
