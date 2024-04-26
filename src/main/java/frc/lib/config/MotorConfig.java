package frc.lib.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Motor config. */
public class MotorConfig {

  /** Use the motor to brake the controller mechanism on neutral output. */
  private boolean neutralBrake = false;

  /** Interpret counterclockwise rotation on the motor face as having positive velocity. */
  private boolean ccwPositive = true;

  /** Ratio between the motor and the mechanism. */
  private double motorToMechanismRatio = 1.0;

  /** Maximum amount of current, in amps, to provide to the motor. */
  private double currentLimitAmps = 40.0;

  /**
   * Modifies this motor config's neutral brake.
   *
   * @param neutralBrake the neutral brake.
   * @return this motor config.
   */
  public MotorConfig withNeutralBrake(boolean neutralBrake) {
    this.neutralBrake = neutralBrake;
    return this;
  }

  /**
   * Modifies this motor config's counterclockwise positive.
   *
   * @param ccwPositive the counterclockwise positive.
   * @return this motor config.
   */
  public MotorConfig withCCWPositive(boolean ccwPositive) {
    this.ccwPositive = ccwPositive;
    return this;
  }

  /**
   * Modifies this motor config's motor to mechanism ratio.
   *
   * @param motorToMechanismRatio the motor to mechanism ratio.
   * @return this motor config.
   */
  public MotorConfig withMotorToMechanismRatio(double motorToMechanismRatio) {
    this.motorToMechanismRatio = motorToMechanismRatio;
    return this;
  }

  /**
   * Modifies this motor config's current limit.
   *
   * @param currentLimitAmps the current limit.
   * @return this motor config.
   */
  public MotorConfig withCurrentLimit(double currentLimitAmps) {
    this.currentLimitAmps = currentLimitAmps;
    return this;
  }

  /**
   * Returns true if motor should neutral brake.
   *
   * @return true if motor should neutral brake.
   */
  public boolean neutralBrake() {
    return neutralBrake;
  }

  /**
   * Returns true if the motor is counterclockwise positive.
   *
   * @return true if the motor is counterclockwise positive.
   */
  public boolean ccwPositive() {
    return ccwPositive;
  }

  /**
   * Returns the motor to mechanism ratio.
   *
   * @return the motor to mechansim ratio.
   */
  public double motorToMechanismRatio() {
    return motorToMechanismRatio;
  }

  /**
   * Returns the motor current limit, in amps.
   *
   * @return the motor current limit, in amps.
   */
  public double currentLimitAmps() {
    return currentLimitAmps;
  }

  /**
   * Creates Phoenix 6 current limit configs.
   *
   * @return Phoenix 6 current limit configs.
   */
  private CurrentLimitsConfigs createCurrentLimitsConfigs() {
    final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    // Stator current is a measure of the current inside of the motor and is typically higher than
    // supply (breaker) current
    currentLimitsConfigs.StatorCurrentLimit = currentLimitAmps() * 2.0;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    currentLimitsConfigs.SupplyCurrentLimit = currentLimitAmps();
    // Allow higher current spikes (150%) for a brief duration (one second)
    // REV 40A auto-resetting breakers typically trip when current exceeds 300% for one second
    currentLimitsConfigs.SupplyCurrentThreshold = currentLimitAmps() * 1.5;
    currentLimitsConfigs.SupplyTimeThreshold = 1;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;

    return currentLimitsConfigs;
  }

  /**
   * Creates Phoenix 6 motor output configs.
   *
   * @return Phoenix 6 motor output configs.
   */
  private MotorOutputConfigs createMotorOutputConfigs() {
    final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    motorOutputConfigs.Inverted =
        ccwPositive() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode =
        neutralBrake() ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    return motorOutputConfigs;
  }

  /**
   * Creates a Phoenix 6 TalonFX config.
   *
   * @return a Phoenix 6 TalonFX config.
   */
  public TalonFXConfiguration createTalonFXConfig() {
    final TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.MotorOutput = createMotorOutputConfigs();

    talonFXConfig.CurrentLimits = createCurrentLimitsConfigs();

    talonFXConfig.Feedback.SensorToMechanismRatio = motorToMechanismRatio();

    return talonFXConfig;
  }
}
