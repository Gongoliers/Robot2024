package frc.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

/** Applies hardware configurations. */
public class ConfigApplier {

  /** Number of retries for configuring Phoenix hardware. */
  private static final int PHOENIX_RETRY_COUNT = 10;

  /**
   * Configures a CANcoder.
   *
   * @param configurator the CANcoder's configurator.
   * @param config the config to apply.
   */
  public static void applyCANcoderConfig(
      CANcoderConfigurator configurator, CANcoderConfiguration config) {
    for (int i = 0; i < PHOENIX_RETRY_COUNT; i++) {
      if (configurator.apply(config) == StatusCode.OK) {
        break;
      }
    }
  }

  /**
   * Configures a TalonFX.
   *
   * @param configurator the TalonFX's configurator.
   * @param config the config to apply.
   */
  public static void applyTalonFXConfig(
      TalonFXConfigurator configurator, TalonFXConfiguration config) {
    for (int i = 0; i < PHOENIX_RETRY_COUNT; i++) {
      if (configurator.apply(config) == StatusCode.OK) {
        break;
      }
    }
  }
}
