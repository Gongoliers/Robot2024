package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.Configurator;

/** Roller motor using a TalonFX. */
public class RollerMotorIOTalonFX implements RollerMotorIO {

  private final TalonFX frontTalonFX, backTalonFX;
  private final SimpleMotorFeedforward topFeedforward, bottomFeedforward;

  public RollerMotorIOTalonFX() {
    frontTalonFX = new TalonFX(50);
    backTalonFX = new TalonFX(40);

    topFeedforward = new SimpleMotorFeedforward(0, 0); // TODO
    bottomFeedforward = new SimpleMotorFeedforward(0, 0); // TODO
  }

  @Override
  public void configure() {
    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 24.0 / 16.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Configurator.configureTalonFX(frontTalonFX.getConfigurator(), config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Configurator.configureTalonFX(backTalonFX.getConfigurator(), config);
  }

  @Override
  public void update(RollerMotorIOValues values) {
    values.velocityRotationsPerSecond = getVelocity();
    values.currentAmps = frontTalonFX.getStatorCurrent().refresh().getValue();
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    double topVolts = topFeedforward.calculate(velocityRotationsPerSecond);

    frontTalonFX.setVoltage(topVolts);

    double bottomVolts = bottomFeedforward.calculate(velocityRotationsPerSecond);

    backTalonFX.setVoltage(bottomVolts);
  }

  public double getVelocity() {
    return frontTalonFX.getVelocity().refresh().getValue();
  }
}
