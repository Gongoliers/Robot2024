package frc.lib;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import java.util.Objects;

/** Constants for motor current limiting. */
public record MotorCurrentLimits(
    double motorAmps, double breakerAmps, double breakerPeakAmps, double peakDurationSeconds) {

  /**
   * Creates motor current limiting constants.
   *
   * @param motorAmps the amount of current allowed in the motor.
   * @param breakerAmps the amount of current allowed at the breaker for an extended duration.
   * @param breakerPeakAmps the peak amount of supply current allowed for a short duration.
   * @param peakDurationSeconds the duration of the current peak.
   */
  public MotorCurrentLimits {
    Objects.requireNonNull(motorAmps);
    Objects.requireNonNull(breakerAmps);
    Objects.requireNonNull(breakerPeakAmps);
    Objects.requireNonNull(peakDurationSeconds);
  }

  /**
   * Creates a Phoenix current limiting configuration using the current limiting constants.
   *
   * @return a Phoenix current limiting configuration.
   */
  public CurrentLimitsConfigs asCurrentLimitsConfigs() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();

    if (motorAmps > 0.0) {
      config.StatorCurrentLimit = motorAmps;
      config.StatorCurrentLimitEnable = true;
    }

    if (breakerAmps > 0.0 && breakerPeakAmps > 0.0 && peakDurationSeconds > 0.0) {
      config.SupplyCurrentLimit = breakerAmps;
      config.SupplyCurrentThreshold = breakerPeakAmps;
      config.SupplyTimeThreshold = peakDurationSeconds;
      config.SupplyCurrentLimitEnable = true;
    }

    return config;
  }
}
