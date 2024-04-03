package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.Configurator;

/** Serializer motor using TalonFX. */
public class SerializerMotorIOTalonFX implements SerializerMotorIO {

  private final TalonFX talonFX;

  private final StatusSignal<Double> velocityRotationsPerSecond, statorCurrentAmps;

  private final SimpleMotorFeedforward velocityFeedforward;

  public SerializerMotorIOTalonFX() {
    talonFX = new TalonFX(42);

    velocityRotationsPerSecond = talonFX.getVelocity();
    statorCurrentAmps = talonFX.getStatorCurrent();

    velocityFeedforward = new SimpleMotorFeedforward(0.14, 0.2617);
  }

  @Override
  public void configure() {
    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 36.0 / 16.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Configurator.configureTalonFX(talonFX.getConfigurator(), config);
  }

  @Override
  public void update(SerializerMotorIOValues values) {
    velocityRotationsPerSecond.refresh();
    statorCurrentAmps.refresh();

    values.velocityRotationsPerSecond = velocityRotationsPerSecond.getValue();
    values.currentAmps = statorCurrentAmps.getValue();
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    double volts = velocityFeedforward.calculate(velocityRotationsPerSecond);

    talonFX.setVoltage(volts);
  }
}
