package frc.lib.config;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import java.util.function.Function;
import java.util.function.Supplier;

/** Applies configs. */
public class ConfigApplier {

  /**
   * Attempts to apply a config. Returns true if successful.
   * 
   * @param applier a function that attempts to apply a config. Returns the result of the application.
   * @param isSuccess a function that returns true if the result of an application is a success.
   * @param retries the number of unsuccessful attempts before failing.
   * @return true if successful.
   */
  private static <Result> boolean apply(Supplier<Result> applier, Function<Result, Boolean> isSuccess, int retries) {
    for (int i = 0; i < retries; i++) {
      Result result = applier.get();

      if (isSuccess.apply(result)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Attempts to apply a Phoenix 6 config. Returns true if successful.
   * 
   * @param applier a function that attempts to apply a config. Returns the result of the application.
   * @return true if successful.
   */
  private static boolean apply(Supplier<StatusCode> applier) {
    return apply(() -> applier.get(), StatusCode::isOK, 10);
  }

  /**
   * Configures a CANcoder.
   *
   * @param configurator the CANcoder's configurator.
   * @param config the config to apply.
   */
  public static void configureCANcoder(
      CANcoderConfigurator configurator, CANcoderConfiguration config) {
    apply(() -> configurator.apply(config));
  }

  /**
   * Configures a TalonFX.
   *
   * @param configurator the TalonFX's configurator.
   * @param config the config to apply.
   */
  public static void configureTalonFX(
      TalonFXConfigurator configurator, TalonFXConfiguration config) {
    apply(() -> configurator.apply(config));
  }

  /**
   * Configures a Pigeon 2.
   *
   * @param configurator the Pigeon 2's configurator.
   * @param config the config to apply.
   */
  public static void configurePigeon2(
      Pigeon2Configurator configurator, Pigeon2Configuration config) {
    apply(() -> configurator.apply(config));
  }

}
