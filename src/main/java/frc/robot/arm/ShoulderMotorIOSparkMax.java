package frc.robot.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.AccelerationCalculator;
import frc.lib.Configurator;
import frc.lib.SingleJointedArmFeedforward;
import frc.robot.arm.ArmConstants.ShoulderMotorConstants;

/** Shoulder motor using a Spark Max. */
public class ShoulderMotorIOSparkMax implements ShoulderMotorIO {

  /** Hardware Spark Max. */
  private final CANSparkMax sparkMax;

  /** Feedback controller for shoulder position. */
  private final PIDController feedback;

  /** Feedforward controller for shoulder position. */
  private final SingleJointedArmFeedforward feedforward;

  private final AccelerationCalculator accelerationCalculator;

  /** Creates a new shoulder motor using a Spark Max. */
  public ShoulderMotorIOSparkMax() {
    sparkMax = new CANSparkMax(2, MotorType.kBrushless);

    feedback = new PIDController(36.0, 0, 0);

    feedforward = new SingleJointedArmFeedforward();

    accelerationCalculator = new AccelerationCalculator();
  }

  @Override
  public void configure() {
    Configurator.configureREV(sparkMax::restoreFactoryDefaults);

    Configurator.configureREV(() -> sparkMax.setIdleMode(IdleMode.kBrake));

    Configurator.configureStatusFrames(sparkMax);
  }

  @Override
  public void update(ShoulderMotorIOValues values) {
    values.positionRotations = getAbsolutePositionRotations();
    values.velocityRotationsPerSecond =
        sparkMax.getEncoder().getVelocity() / ShoulderMotorConstants.JOINT_CONSTANTS.gearing();
    values.accelerationRotationsPerSecondPerSecond =
        accelerationCalculator.calculate(values.velocityRotationsPerSecond);

    values.currentAmps = sparkMax.getOutputCurrent();
    values.inputVoltage = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
  }

  @Override
  public void setPosition(double positionRotations) {
    sparkMax
        .getEncoder()
        .setPosition(positionRotations * ShoulderMotorConstants.JOINT_CONSTANTS.gearing());
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = getAbsolutePositionRotations();

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts =
        feedforward.calculate(
            Rotation2d.fromRotations(measuredPositionRotations),
            Rotation2d.fromRotations(velocityRotationsPerSecond));

    setVoltage(feedbackVolts + feedforwardVolts);
  }

  /**
   * Gets the absolute position of the shoulder in rotations.
   *
   * @return the absolute position of the shoulder in rotations.
   */
  private double getAbsolutePositionRotations() {
    return sparkMax.getEncoder().getPosition() / ShoulderMotorConstants.JOINT_CONSTANTS.gearing();
  }

  @Override
  public void setVoltage(double volts) {
    sparkMax.setVoltage(volts);
  }
}
