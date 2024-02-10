package frc.robot.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.ArmFeedforwardCalculator;
import frc.lib.Configurator;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Shoulder motor using a Spark Max. */
public class ShoulderMotorIOSparkMax implements ShoulderMotorIO {

  /** Hardware Spark Max. */
  private final CANSparkMax sparkMax;

  /** Feedback controller for shoulder position. */
  private final PIDController feedback;

  /** Feedforward controller for shoulder position. */
  private final ArmFeedforward feedforward;

  /** Creates a new shoulder motor using a Spark Max. */
  public ShoulderMotorIOSparkMax() {
    sparkMax = new CANSparkMax(ShoulderMotorConstants.CAN.id(), MotorType.kBrushless);

    feedback = new PIDController(ShoulderMotorConstants.KP, 0, 0);

    feedforward =
        new ArmFeedforward(
            0,
            ArmFeedforwardCalculator.calculateArmGravityCompensation(
                Rotation2d.fromDegrees(18.0), 0.1222),
            0);
  }

  @Override
  public void configure() {
    Configurator.configureREV(sparkMax::restoreFactoryDefaults);

    Configurator.configureREV(() -> sparkMax.setIdleMode(IdleMode.kBrake));
  }

  @Override
  public void update(ShoulderMotorIOValues values) {
    values.positionRotations = getPositionRotations();
    values.currentAmps = sparkMax.getOutputCurrent();
  }

  @Override
  public void setPosition(double positionRotations) {
    sparkMax.getEncoder().setPosition(positionRotations * ShoulderMotorConstants.GEARING);
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = getPositionRotations();

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts =
        feedforward.calculate(measuredPositionRotations, velocityRotationsPerSecond);

    sparkMax.setVoltage(feedbackVolts + feedforwardVolts);
  }

  /**
   * Gets the absolute position of the shoulder in rotations.
   *
   * @return the absolute position of the shoulder in rotations.
   */
  private double getPositionRotations() {
    return sparkMax.getEncoder().getPosition() / ShoulderMotorConstants.GEARING;
  }
}
