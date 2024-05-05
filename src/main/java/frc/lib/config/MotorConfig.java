package frc.lib.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Objects;

/** Motor config. */
public record MotorConfig(
    boolean neutralBrake,
    boolean ccwPositive,
    double motorToMechanismRatio,
    double statorCurrentLimit,
    double supplyCurrentLimit) {

  public MotorConfig {
    Objects.requireNonNull(neutralBrake);
    Objects.requireNonNull(ccwPositive);
    Objects.requireNonNull(motorToMechanismRatio);
    Objects.requireNonNull(statorCurrentLimit);
    Objects.requireNonNull(supplyCurrentLimit);
  }

  public static final class MotorConfigBuilder {
    private boolean neutralBrake;

    private boolean ccwPositive;

    private double motorToMechanismRatio;

    private double statorCurrentLimit;

    private double supplyCurrentLimit;

    public static MotorConfigBuilder defaults() {
      return new MotorConfigBuilder(false, true, 1.0, 80.0, 40.0);
    }

    public static MotorConfigBuilder from(MotorConfig motorConfig) {
      return new MotorConfigBuilder(
          motorConfig.neutralBrake,
          motorConfig.ccwPositive,
          motorConfig.motorToMechanismRatio,
          motorConfig.statorCurrentLimit,
          motorConfig.supplyCurrentLimit);
    }

    private MotorConfigBuilder(
        boolean neutralBrake,
        boolean ccwPositive,
        double motorToMechanismRatio,
        double statorCurrentLimit,
        double supplyCurrentLimit) {
      this.neutralBrake = neutralBrake;
      this.ccwPositive = ccwPositive;
      this.motorToMechanismRatio = motorToMechanismRatio;
      this.statorCurrentLimit = statorCurrentLimit;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public MotorConfigBuilder neutralBrake(boolean neutralBrake) {
      this.neutralBrake = neutralBrake;
      return this;
    }

    public MotorConfigBuilder ccwPositive(boolean ccwPositive) {
      this.ccwPositive = ccwPositive;
      return this;
    }

    public MotorConfigBuilder motorToMechanismRatio(double motorToMechanismRatio) {
      this.motorToMechanismRatio = motorToMechanismRatio;
      return this;
    }

    public MotorConfigBuilder statorCurrentLimit(double statorCurrentLimit) {
      this.statorCurrentLimit = statorCurrentLimit;
      return this;
    }

    public MotorConfigBuilder supplyCurrentLimit(double supplyCurrentLimit) {
      this.supplyCurrentLimit = supplyCurrentLimit;
      return this;
    }

    public MotorConfig build() {
      return new MotorConfig(
          neutralBrake, ccwPositive, motorToMechanismRatio, statorCurrentLimit, supplyCurrentLimit);
    }
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
    currentLimitsConfigs.StatorCurrentLimit = statorCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    currentLimitsConfigs.SupplyCurrentLimit = supplyCurrentLimit;
    // TODO Determine if spikes should be eliminated to preserve battery
    // Allow higher current spikes (150%) for a very brief duration (tenth second)
    currentLimitsConfigs.SupplyCurrentThreshold = supplyCurrentLimit * 1.5;
    currentLimitsConfigs.SupplyTimeThreshold = 0.1;
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
