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
import frc.robot.arm.ArmState;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;

/** Helper class for rendering robot mechanisms. */
public class RobotMechanisms {

  private static RobotMechanisms instance = null;

  private final Mechanism2d mechanism;

  private MechanismLigament2d shoulder, wrist, intake;

  private final double WIDTH =
      2
          * (RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE
              + RobotConstants.MAX_HORIZONTAL_EXTENSION_DISTANCE);

  private final double HEIGHT = RobotConstants.MAX_VERTICAL_EXTENSION_DISTANCE;

  private final Translation2d ORIGIN = new Translation2d(WIDTH / 2, 0);

  private RobotMechanisms() {
    mechanism = new Mechanism2d(WIDTH, HEIGHT);

    initializeArmMechanism();
    initializeFramePerimeterMechanisms();
    initializeIntakeMechanism();
  }

  private void initializeArmMechanism() {
    Translation2d armRootTranslation = ORIGIN.plus(ShoulderMotorConstants.SHOULDER_TO_ORIGIN);

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
                new Color8Bit(Color.kOrange)));
    wrist =
        shoulder.append(
            new MechanismLigament2d(
                "wrist",
                // WristMotorConstants.JOINT_CONSTANTS.lengthMeters(),
                0.0,
                0,
                armThickness,
                new Color8Bit(Color.kGreen)));
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
                "framePerimeterLeft_",
                HEIGHT,
                90,
                framePerimeterThickness,
                new Color8Bit(Color.kLightGray)));

    mechanism
        .getRoot(
            "framePerimeterRight",
            ORIGIN.getX() + RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE,
            0)
        .append(
            new MechanismLigament2d(
                "framePerimeterRight_",
                HEIGHT,
                90,
                framePerimeterThickness,
                new Color8Bit(Color.kLightGray)));
  }

  private void initializeIntakeMechanism() {
    double intakeThickness = Units.inchesToMeters(3) * 100;

    intake =
        mechanism
            .getRoot(
                "intake",
                ORIGIN.getX() + Units.inchesToMeters(13.164),
                ORIGIN.getY() + Units.inchesToMeters(6.283))
            .append(
                new MechanismLigament2d(
                    "intake_",
                    PivotMotorConstants.DISTANCE,
                    0.0,
                    intakeThickness,
                    new Color8Bit(Color.kPink)));
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

  public void setArmState(ArmState state) {
    Rotation2d shoulderRotation = Rotation2d.fromRotations(state.shoulder().position);
    Rotation2d wristRotation = Rotation2d.fromRotations(state.wrist().position);

    shoulder.setAngle(shoulderRotation);
    wrist.setAngle(wristRotation.minus(shoulderRotation));
  }

  public void setIntakeAngle(Rotation2d angle) {
    intake.setAngle(angle);
  }
}
