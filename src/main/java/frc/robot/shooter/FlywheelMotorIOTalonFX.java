package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    talonFX.setNeutralMode(NeutralModeValue.Coast);
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
