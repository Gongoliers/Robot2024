package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotConstants;
import frc.robot.arm.ArmConstants.ElbowMotorConstants;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Helper class for rendering the arm using WPILib mechanisms. */
public class ArmMechanism {

  private static ArmMechanism instance = null;

  private final Mechanism2d mechanism;

  private final MechanismRoot2d root;

  private final MechanismLigament2d shoulder, elbow;

  private final double THICKNESS = Units.inchesToMeters(2) * 100;

  private ArmMechanism() {
    mechanism =
        new Mechanism2d(
            2
                * (RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE
                    + RobotConstants.MAX_HORIZONTAL_EXTENSION_DISTANCE),
            RobotConstants.MAX_VERTICAL_EXTENSION_DISTANCE);
    root =
        mechanism.getRoot(
            "arm",
            RobotConstants.FRAME_PERIMETER_TO_ORIGIN_DISTANCE
                + RobotConstants.MAX_HORIZONTAL_EXTENSION_DISTANCE
                - ShoulderMotorConstants.SHOULDER_TRANSLATION.getX(),
            ShoulderMotorConstants.SHOULDER_TRANSLATION.getY());
    shoulder =
        root.append(
            new MechanismLigament2d(
                "shoulder",
                ShoulderMotorConstants.SHOULDER_TO_ELBOW_DISTANCE,
                0,
                THICKNESS,
                new Color8Bit(Color.kOrange)));
    elbow =
        shoulder.append(
            new MechanismLigament2d(
                "elbow",
                ElbowMotorConstants.ELBOW_TO_WRIST_DISTANCE,
                0,
                THICKNESS,
                new Color8Bit(Color.kGreen)));
  }

  public static ArmMechanism getInstance() {
    if (instance == null) {
      instance = new ArmMechanism();
    }

    return instance;
  }

  public Mechanism2d getMechanism() {
    return mechanism;
  }

  public void setState(ArmState state) {
    Rotation2d shoulderRotation = Rotation2d.fromRotations(state.shoulder().position);
    Rotation2d elbowRotation = Rotation2d.fromRotations(state.elbow().position);
    // Rotation2d wristRotation = Rotation2d.fromRotations(state.wrist().position);

    shoulder.setAngle(shoulderRotation);
    elbow.setAngle(elbowRotation.minus(shoulderRotation));
  }
}
