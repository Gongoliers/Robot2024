package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.Configurator;

/** Flywheel motor using TalonFX. */
public class FlywheelMotorIOTalonFX implements FlywheelMotorIO {

  private final TalonFX talonFX;

  private final StatusSignal<Double> velocityRotationsPerSecond, statorCurrentAmps;

  private final SimpleMotorFeedforward velocityFeedforward;

  public FlywheelMotorIOTalonFX() {
    talonFX = new TalonFX(44);

    velocityRotationsPerSecond = talonFX.getVelocity();
    statorCurrentAmps = talonFX.getStatorCurrent();

    velocityFeedforward = new SimpleMotorFeedforward(0, 0);
  }

  @Override
  public void configure() {
    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 36.0 / 16.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Configurator.configureTalonFX(talonFX.getConfigurator(), config);
  }

  @Override
  public void update(FlywheelMotorIOValues values) {
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
