package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.Configurator;

/** Flywheel motor using TalonFX. */
public class FlywheelMotorIOTalonFX implements FlywheelMotorIO {

  private final TalonFX talonFX;

  private final StatusSignal<Double> velocityRotationsPerSecond, statorCurrentAmps;

  public FlywheelMotorIOTalonFX() {
    talonFX = new TalonFX(30);

    velocityRotationsPerSecond = talonFX.getVelocity();
    statorCurrentAmps = talonFX.getStatorCurrent();
  }

  @Override
  public void configure() {
    final TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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
    // TODO Implement velocity setpoint
  }
}
