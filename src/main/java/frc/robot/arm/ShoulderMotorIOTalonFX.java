package frc.robot.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.Configurator;

/** Shoulder motor using a TalonFX. */
public class ShoulderMotorIOTalonFX implements ShoulderMotorIO {

  private final TalonFX eastTalonFX, westTalonFX;

  /** Feedback controller for shoulder position. */
  private final PIDController feedback;

  /** Feedforward controller for shoulder position. */
  private final ArmFeedforward feedforward;

  /** Creates a new shoulder motor using a Spark Max. */
  public ShoulderMotorIOTalonFX() {
    eastTalonFX = new TalonFX(0); // TODO
    westTalonFX = new TalonFX(0); // TODO

    feedback = new PIDController(0, 0, 0);

    feedforward = new ArmFeedforward(0, 0, 0);
  }

  @Override
  public void configure() {
    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 1.0; // TODO

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    Configurator.configureTalonFX(eastTalonFX.getConfigurator(), config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    Configurator.configureTalonFX(westTalonFX.getConfigurator(), config);
  }

  @Override
  public void update(ShoulderMotorIOValues values) {
    values.positionRotations = getAbsolutePositionRotations();
    values.velocityRotationsPerSecond = westTalonFX.getVelocity().refresh().getValue();
    values.accelerationRotationsPerSecondPerSecond = westTalonFX.getAcceleration().refresh().getValue();

    values.currentAmps = westTalonFX.getStatorCurrent().refresh().getValue();
    values.inputVoltage = westTalonFX.getMotorVoltage().refresh().getValue();
  }

  @Override
  public void setPosition(double positionRotations) {
    eastTalonFX.setPosition(positionRotations);
    westTalonFX.setPosition(positionRotations);
  }

  @Override
  public void setSetpoint(double positionRotations, double velocityRotationsPerSecond) {
    double measuredPositionRotations = getAbsolutePositionRotations();

    double feedbackVolts = feedback.calculate(measuredPositionRotations, positionRotations);

    double feedforwardVolts =
        feedforward.calculate(Units.rotationsToRadians(measuredPositionRotations), velocityRotationsPerSecond);

    setVoltage(feedbackVolts + feedforwardVolts);
  }

  /**
   * Gets the absolute position of the shoulder in rotations.
   *
   * @return the absolute position of the shoulder in rotations.
   */
  private double getAbsolutePositionRotations() {
    return eastTalonFX.getPosition().refresh().getValue();
  }

  @Override
  public void setVoltage(double volts) {
    eastTalonFX.setVoltage(volts);
    westTalonFX.setVoltage(volts);
  }
}
