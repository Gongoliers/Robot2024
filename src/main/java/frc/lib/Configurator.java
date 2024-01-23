package frc.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.revrobotics.REVLibError;
import java.util.function.Supplier;

/** Applies hardware configurations. */
public class Configurator {

  /** Number of retries for configuring Phoenix hardware. */
  private static final int PHOENIX_RETRY_COUNT = 10;

  /** Number of retries for configuring REV hardware. */
  private static final int REV_RETRY_COUNT = 10;

  /**
   * Configures a Phoenix device.
   *
   * @param config a lambda that configures a Phoenix device.
   */
  public static void configurePhoenix(Supplier<StatusCode> config) {
    for (int i = 0; i < PHOENIX_RETRY_COUNT; i++) {
      if (config.get() == StatusCode.OK) {
        return;
      }
    }
  }

  /**
   * Configures a CANcoder.
   *
   * @param configurator the CANcoder's configurator.
   * @param config the config to apply.
   */
  public static void configureCANcoder(
      CANcoderConfigurator configurator, CANcoderConfiguration config) {
    configurePhoenix(() -> configurator.apply(config));
  }

  /**
   * Configures a TalonFX.
   *
   * @param configurator the TalonFX's configurator.
   * @param config the config to apply.
   */
  public static void configureTalonFX(
      TalonFXConfigurator configurator, TalonFXConfiguration config) {
    configurePhoenix(() -> configurator.apply(config));
  }

  /**
   * Configures a Pigeon 2.
   *
   * @param configurator the Pigeon 2's configurator.
   * @param config the config to apply.
   */
  public static void configurePigeon2(
      Pigeon2Configurator configurator, Pigeon2Configuration config) {
    configurePhoenix(() -> configurator.apply(config));
  }

  /**
   * Configures a REV device.
   *
   * @param config a lambda that configures a REV device.
   */
  public static void configureREV(Supplier<REVLibError> config) {
    for (int i = 0; i < REV_RETRY_COUNT; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
  }
}
