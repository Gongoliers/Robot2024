package frc.lib;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import java.util.function.Supplier;

/** Applies hardware configurations. */
public class Configurator {

  /** Number of retries for configuring Phoenix hardware. */
  private static final int PHOENIX_RETRY_COUNT = 10;

  /** Number of retries for configuring REV hardware. */
  private static final int REV_RETRY_COUNT = 10;

  /**
   * Configures a Phoenix 6 device.
   *
   * @param config a lambda that configures a Phoenix 6 device.
   */
  public static void configurePhoenix6(Supplier<StatusCode> config) {
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
    configurePhoenix6(() -> configurator.apply(config));
  }

  /**
   * Configures a TalonFX.
   *
   * @param configurator the TalonFX's configurator.
   * @param config the config to apply.
   */
  public static void configureTalonFX(
      TalonFXConfigurator configurator, TalonFXConfiguration config) {
    configurePhoenix6(() -> configurator.apply(config));
  }

  /**
   * Configures a Pigeon 2.
   *
   * @param configurator the Pigeon 2's configurator.
   * @param config the config to apply.
   */
  public static void configurePigeon2(
      Pigeon2Configurator configurator, Pigeon2Configuration config) {
    configurePhoenix6(() -> configurator.apply(config));
  }

  /**
   * Configures a Phoenix 5 device.
   *
   * @param config a lambda that configures a Phoenix 5 device.
   */
  public static void configurePhoenix5(Supplier<ErrorCode> config) {
    for (int i = 0; i < PHOENIX_RETRY_COUNT; i++) {
      if (config.get() == ErrorCode.OK) {
        return;
      }
    }
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

  /**
   * Configures the status frames for a Spark Max.
   *
   * @param sparkMax
   */
  public static void configureStatusFrames(CANSparkMax sparkMax) {
    Configurator.configureREV(() -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10));
    Configurator.configureREV(() -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 25));
    Configurator.configureREV(() -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 25));
    Configurator.configureREV(() -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    Configurator.configureREV(() -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    Configurator.configureREV(() -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500));
    Configurator.configureREV(() -> sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500));
  }
}
