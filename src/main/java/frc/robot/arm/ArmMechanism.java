package frc.robot.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
    mechanism = new Mechanism2d(1, 1);
    root = mechanism.getRoot("arm", 0.15, 0);
    shoulder =
        root.append(
            new MechanismLigament2d(
                "shoulder",
                ShoulderMotorConstants.SHOULDER_TO_ELBOW_DISTANCE,
                90,
                THICKNESS,
                new Color8Bit(Color.kOrange)));
    elbow =
        shoulder.append(
            new MechanismLigament2d(
                "elbow",
                ElbowMotorConstants.ELBOW_TO_WRIST_DISTANCE,
                90,
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
    shoulder.setAngle(state.shoulder());
    elbow.setAngle(state.elbow().minus(state.shoulder()));
  }
}
