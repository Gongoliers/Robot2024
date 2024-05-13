package frc.lib.config;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Function;
import java.util.function.Supplier;

/** Applies configs. */
public class ConfigApplier {

  /**
   * Attempts to apply a config. Returns true if successful.
   *
   * @param applier a function that attempts to apply a config. Returns the result of the
   *     application.
   * @param isSuccess a function that returns true if the result of an application is a success.
   * @param retries the number of unsuccessful attempts before failing.
   * @return true if successful.
   */
  private static <Result> boolean attempt(
      Supplier<Result> applier, Function<Result, Boolean> isSuccess, int retries) {
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
   * @param applier a function that attempts to apply a config. Returns the result of the
   *     application.
   * @return true if successful.
   */
  private static boolean attempt(Supplier<StatusCode> applier) {
    return attempt(() -> applier.get(), StatusCode::isOK, 10);
  }

  /**
   * Attempts to apply a CANcoder config. Warns on failure.
   *
   * @param cancoder the CANcoder to configure.
   * @param config the config to apply.
   * @return true if successful.
   */
  public static boolean applyCANcoderConfig(CANcoder cancoder, CANcoderConfiguration config) {
    CANcoderConfigurator configurator = cancoder.getConfigurator();

    boolean success = attempt(() -> configurator.apply(config));

    if (!success) {
      DriverStation.reportWarning(
          "Failed to apply config for CANcoder ID: " + cancoder.getDeviceID(), false);
    }

    return success;
  }

  /**
   * Attempts to apply a TalonFX config. Warns on failure.
   *
   * @param talonFX the TalonFX to configure.
   * @param config the config to apply.
   * @return true if successful.
   */
  public static boolean applyTalonFXConfig(TalonFX talonFX, TalonFXConfiguration config) {
    TalonFXConfigurator configurator = talonFX.getConfigurator();

    boolean success = attempt(() -> configurator.apply(config));

    if (!success) {
      DriverStation.reportWarning(
          "Failed to apply config for TalonFX ID: " + talonFX.getDeviceID(), false);
    }

    return success;
  }

  /**
   * Attempts to apply a Pigeon 2 config. Warns on failure.
   *
   * @param pigeon2 the Pigeon 2 to configure.
   * @param config the config to apply.
   * @return true if successful.
   */
  public static boolean applyPigeon2Config(Pigeon2 pigeon2, Pigeon2Configuration config) {
    Pigeon2Configurator configurator = pigeon2.getConfigurator();

    boolean success = attempt(() -> configurator.apply(config));

    if (!success) {
      DriverStation.reportWarning(
          "Failed to apply config for Pigeon 2 ID: " + pigeon2.getDeviceID(), false);
    }

    return success;
  }
}
