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
    double currentLimitAmps) {

  public MotorConfig {
    Objects.requireNonNull(neutralBrake);
    Objects.requireNonNull(ccwPositive);
    Objects.requireNonNull(motorToMechanismRatio);
    Objects.requireNonNull(currentLimitAmps);
  }

  public static final class MotorConfigBuilder {
    private boolean neutralBrake;

    private boolean ccwPositive;

    private double motorToMechanismRatio;

    private double currentLimitAmps;

    public static MotorConfigBuilder defaults() {
      return new MotorConfigBuilder(false, true, 1.0, 40.0);
    }

    public static MotorConfigBuilder from(MotorConfig motorConfig) {
      return new MotorConfigBuilder(
          motorConfig.neutralBrake,
          motorConfig.ccwPositive,
          motorConfig.motorToMechanismRatio,
          motorConfig.currentLimitAmps);
    }

    private MotorConfigBuilder(
        boolean neutralBrake,
        boolean ccwPositive,
        double motorToMechanismRatio,
        double currentLimitAmps) {
      this.neutralBrake = neutralBrake;
      this.ccwPositive = ccwPositive;
      this.motorToMechanismRatio = motorToMechanismRatio;
      this.currentLimitAmps = currentLimitAmps;
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

    public MotorConfigBuilder currentLimitAmps(double currentLimitAmps) {
      this.currentLimitAmps = currentLimitAmps;
      return this;
    }

    public MotorConfig build() {
      return new MotorConfig(neutralBrake, ccwPositive, motorToMechanismRatio, currentLimitAmps);
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
