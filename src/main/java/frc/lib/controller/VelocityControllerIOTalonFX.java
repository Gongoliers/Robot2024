package frc.lib.controller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.CAN;
import frc.lib.config.Configurator;
import frc.lib.config.MechanismConfig;

/** Velocity controller using TalonFX. */
public abstract class VelocityControllerIOTalonFX implements VelocityControllerIO {

  protected final MechanismConfig config;

  protected final TalonFX motor;

  protected final StatusSignal<Double> position, velocity, acceleration, volts, amps;

  /**
   * Creates a new velocity controller using TalonFX.
   *
   * @param can
   */
  protected VelocityControllerIOTalonFX(CAN can, MechanismConfig config) {
    this.config = config;

    motor = new TalonFX(can.id(), can.bus());

    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    volts = motor.getMotorVoltage();
    amps = motor.getStatorCurrent();
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity, acceleration, volts, amps);

    ParentDevice.optimizeBusUtilizationForAll(motor);

    Configurator.configureTalonFX(
        motor.getConfigurator(), config.motorConfig().createTalonFXConfig());
  }

  @Override
  public void update(VelocityControllerIOValues values) {
    BaseStatusSignal.refreshAll(position, velocity, acceleration, volts, amps);

    values.positionRotations = position.getValue();
    values.velocityRotationsPerSecond = velocity.getValue();
    values.accelerationRotationsPerSecondPerSecond = acceleration.getValue();
    values.motorVolts = volts.getValue();
    values.motorAmps = amps.getValue();
  }

  @Override
  public void setPosition(double positionRotations) {
    motor.setPosition(positionRotations);
  }

  @Override
  public abstract void setSetpoint(double velocityRotationsPerSecond);
}
