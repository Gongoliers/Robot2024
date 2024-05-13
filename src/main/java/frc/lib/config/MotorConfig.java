package frc.lib.config;

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
}
