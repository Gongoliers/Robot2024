package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.AccelerationCalculator;
import frc.lib.Configurator;
import frc.lib.SingleJointedArmFeedforward;
import frc.robot.arm.ArmConstants.WristMotorConstants;

public class WristMotorIOSparkMax implements WristMotorIO {

    private final CANSparkMax sparkMax;

  /** Feedback controller for wrist position. */
  private final PIDController feedback;

  /** Feedforward controller for wrist position. */
  private final SingleJointedArmFeedforward feedforward;

  private final AccelerationCalculator accelerationCalculator;

    public WristMotorIOSparkMax() {
        sparkMax = new CANSparkMax(WristMotorConstants.CAN.id(), MotorType.kBrushless);

    feedback = new PIDController(WristMotorConstants.KP, 0, 0);

    feedforward =
        new SingleJointedArmFeedforward(
            0,
            0,
            0);

        accelerationCalculator = new AccelerationCalculator();
    }

    @Override
    public void configure() {
        Configurator.configureREV(sparkMax::restoreFactoryDefaults);

        // TODO
        sparkMax.setInverted(true);

        Configurator.configureREV(() -> sparkMax.setIdleMode(IdleMode.kBrake));
    }

    @Override
    public void update(WristMotorIOValues values) {
    values.positionRotations = getPositionRotations();
    values.velocityRotationsPerSecond =
        sparkMax.getEncoder().getVelocity() / WristMotorConstants.JOINT_CONSTANTS.gearing();
    values.accelerationRotationsPerSecondPerSecond =
        accelerationCalculator.calculate(values.velocityRotationsPerSecond);

    values.currentAmps = sparkMax.getOutputCurrent();
    values.appliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    }

    @Override
    public void setPosition(double positionRotations) {
    sparkMax
        .getEncoder()
        .setPosition(positionRotations * WristMotorConstants.JOINT_CONSTANTS.gearing());
    }

    @Override
    public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = getPositionRotations();

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts =
        feedforward.calculate(Rotation2d.fromRotations(measuredPositionRotations), velocityRotationsPerSecond);

    setVoltage(feedbackVolts + feedforwardVolts);
    }

    @Override
    public void setVoltage(double volts) {
    sparkMax.setVoltage(volts);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }

  /**
   * Gets the absolute position of the wrist in rotations.
   *
   * @return the absolute position of the wrist in rotations.
   */
  private double getPositionRotations() {
    return sparkMax.getEncoder().getPosition() / WristMotorConstants.JOINT_CONSTANTS.gearing();
  }
    
}
