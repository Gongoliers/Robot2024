package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.Configurator;

/** Roller motor using a TalonFX. */
public class RollerMotorIOTalonFX implements RollerMotorIO {

  private final TalonFX topTalonFX, bottomTalonFX;
  private final SimpleMotorFeedforward topFeedforward, bottomFeedforward;

  public RollerMotorIOTalonFX() {
    topTalonFX = new TalonFX(0); // TODO
    bottomTalonFX = new TalonFX(0); // TODO

    topFeedforward = new SimpleMotorFeedforward(0, 0); // TODO
    bottomFeedforward = new SimpleMotorFeedforward(0, 0); // TODO
  }

  @Override
  public void configure() {
    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 1.0; // TODO

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    Configurator.configureTalonFX(topTalonFX.getConfigurator(), config);
    Configurator.configureTalonFX(bottomTalonFX.getConfigurator(), config);
  }

  @Override
  public void update(RollerMotorIOValues values) {
    values.velocityRotationsPerSecond = getVelocity();
    values.currentAmps = topTalonFX.getStatorCurrent().refresh().getValue();
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    double topVolts = topFeedforward.calculate(velocityRotationsPerSecond);

    topTalonFX.setVoltage(topVolts);

    double bottomVolts = bottomFeedforward.calculate(velocityRotationsPerSecond);

    bottomTalonFX.setVoltage(bottomVolts);
  }

  public double getVelocity() {
    return topTalonFX.getVelocity().refresh().getValue();
  }
}
