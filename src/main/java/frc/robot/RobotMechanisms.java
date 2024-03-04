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
import frc.lib.InterpolatableColor;
import frc.robot.arm.ArmState;
import frc.robot.intake.IntakeConstants.PivotMotorConstants;
import frc.robot.intake.IntakeConstants.RollerMotorConstants;
import frc.robot.shooter.ShooterConstants.FlywheelConstants;
import frc.robot.shooter.ShooterConstants.SerializerConstants;

/** Helper class for rendering robot mechanisms. */
public class RobotMechanisms {

  private static RobotMechanisms instance = null;

  private final Mechanism2d mechanism;

  private MechanismLigament2d shoulder, shooter, serializer, intake;

  private final double WIDTH =
      2
          * (RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE
              + RobotConstants.MAX_HORIZONTAL_EXTENSION_DISTANCE);

  private final double HEIGHT = RobotConstants.MAX_VERTICAL_EXTENSION_DISTANCE;

  private final Translation2d ORIGIN = new Translation2d(WIDTH / 2, 0);

  private final Color8Bit DEFAULT_COLOR = new Color8Bit(Color.kLightGray);

  private final InterpolatableColor shooterColor = new InterpolatableColor(Color.kLightGray, Color.kSalmon);
  private final InterpolatableColor serializerColor = new InterpolatableColor(Color.kLightGray, Color.kCornflowerBlue);
  private final InterpolatableColor intakeColor = new InterpolatableColor(Color.kLightGray, Color.kPaleGreen);

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
                DEFAULT_COLOR));
    shooter =
        shoulder.append(
            new MechanismLigament2d(
                "wrist",
                Units.inchesToMeters(4.321),
                0,
                armThickness,
                DEFAULT_COLOR));

    serializer =
        shoulder.append(
            new MechanismLigament2d(
                "serializer",
                Units.inchesToMeters(8.771),
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
                "framePerimeterLeft_",
                HEIGHT,
                90,
                framePerimeterThickness,
                DEFAULT_COLOR));

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
                DEFAULT_COLOR));
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
                    DEFAULT_COLOR));
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

  public void updateArm(ArmState state) {
    Rotation2d shoulderRotation = Rotation2d.fromRotations(state.shoulder().position);
    Rotation2d wristRotation = Rotation2d.fromRotations(state.wrist().position);

    shoulder.setAngle(shoulderRotation);

    // Offset the rendered wrist rotation so that zero degrees is perpendicular to the shoulder, rather than parallel with the shoulder
    Rotation2d offsetWristRotation = wristRotation.plus(Rotation2d.fromDegrees(90));
    Rotation2d shooterRotation = offsetWristRotation;
    Rotation2d serializerRotation = offsetWristRotation.plus(Rotation2d.fromDegrees(180));

    shooter.setAngle(shooterRotation);
    serializer.setAngle(serializerRotation);
  }

  public void updateIntake(Rotation2d pivotAngle, double rollerVelocityRotationsPerSecond) {
    Color color = intakeColor.sample(Math.abs(rollerVelocityRotationsPerSecond), 0, RollerMotorConstants.MAXIMUM_SPEED);

    intake.setAngle(pivotAngle);
    intake.setColor(new Color8Bit(color));
  }

  public void updateShooter(double velocityMetersPerSecond) {
    Color color = shooterColor.sample(Math.abs(velocityMetersPerSecond), 0, FlywheelConstants.MAXIMUM_TANGENTIAL_SPEED);

    shooter.setColor(new Color8Bit(color));
  }

  public void updateSerializer(double velocityMetersPerSecond) {
    Color color = serializerColor.sample(Math.abs(velocityMetersPerSecond), 0, SerializerConstants.MAXIMUM_TANGENTIAL_SPEED);

    serializer.setColor(new Color8Bit(color));
  }
}
