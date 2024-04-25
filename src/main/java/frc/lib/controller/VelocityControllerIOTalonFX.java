package frc.lib.controller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.CAN;
import frc.lib.Configurator;
import frc.lib.config.MechanismConfig;

/** Velocity controller using TalonFX. */
public abstract class VelocityControllerIOTalonFX implements VelocityControllerIO {

  protected final MechanismConfig config;

  protected final TalonFX motor;

  protected final StatusSignal<Double> velocity, acceleration, volts, amps;

  /**
   * Creates a new velocity controller using TalonFX.
   *
   * @param can
   */
  protected VelocityControllerIOTalonFX(CAN can, MechanismConfig config) {
    this.config = config;
    
    motor = new TalonFX(can.id(), can.bus());

    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    volts = motor.getMotorVoltage();
    amps = motor.getStatorCurrent();
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, velocity, acceleration, volts, amps);

    ParentDevice.optimizeBusUtilizationForAll(motor);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted =
        config.motor.ccwPositive()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode =
        config.motor.neutralBrake() ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Stator current is a measure of the current inside of the motor and is typically higher than
    // supply (breaker) current
    motorConfig.CurrentLimits.StatorCurrentLimit = config.motor.currentLimitAmps() * 2.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.CurrentLimits.SupplyCurrentLimit = config.motor.currentLimitAmps();
    // Allow higher current spikes (150%) for a brief duration (one second)
    // REV 40A auto-resetting breakers typically trip when current exceeds 300% for one second
    motorConfig.CurrentLimits.SupplyCurrentThreshold = config.motor.currentLimitAmps() * 1.5;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 1;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Feedback.SensorToMechanismRatio = config.motor.motorToMechanismRatio();

    Configurator.configureTalonFX(motor.getConfigurator(), motorConfig);
  }

  @Override
  public void update(VelocityControllerIOValues values) {
    BaseStatusSignal.refreshAll(velocity, acceleration, volts, amps);

    values.velocityRotationsPerSecond = velocity.getValue();
    values.accelerationRotationsPerSecondPerSecond = acceleration.getValue();
    values.motorVolts = volts.getValue();
    values.motorAmps = amps.getValue();
  }

  @Override
  public abstract void setSetpoint(double velocityRotationsPerSecond);
}
