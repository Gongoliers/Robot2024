package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Objects;

/** Absolute encoder config. */
public record AbsoluteEncoderConfig(
    boolean ccwPositive, double sensorToMechanismRatio, Rotation2d offset) {

  public AbsoluteEncoderConfig {
    Objects.requireNonNull(ccwPositive);
    Objects.requireNonNull(sensorToMechanismRatio);
    Objects.requireNonNull(offset);
  }

  public static final class AbsoluteEncoderConfigBuilder {
    /** Interpret counterclockwise rotation on the encoder as having positive velocity. */
    private boolean ccwPositive;

    /** Ratio between the absolute encoder and the mechanism. */
    private double sensorToMechanismRatio;

    /** Offset between absolute encoder reading and mechanism position. */
    private Rotation2d offset;

    public static AbsoluteEncoderConfigBuilder defaults() {
      return new AbsoluteEncoderConfigBuilder(true, 1.0, new Rotation2d());
    }

    public static AbsoluteEncoderConfigBuilder from(AbsoluteEncoderConfig absoluteEncoderConfig) {
      return new AbsoluteEncoderConfigBuilder(
          absoluteEncoderConfig.ccwPositive,
          absoluteEncoderConfig.sensorToMechanismRatio,
          absoluteEncoderConfig.offset);
    }

    private AbsoluteEncoderConfigBuilder(
        boolean ccwPositive, double sensorToMechanismRatio, Rotation2d offset) {
      this.ccwPositive = ccwPositive;
      this.sensorToMechanismRatio = sensorToMechanismRatio;
      this.offset = offset;
    }

    public AbsoluteEncoderConfigBuilder ccwPositive(boolean ccwPositive) {
      this.ccwPositive = ccwPositive;
      return this;
    }

    public AbsoluteEncoderConfigBuilder sensorToMechanismRatio(double sensorToMechanismRatio) {
      this.sensorToMechanismRatio = sensorToMechanismRatio;
      return this;
    }

    public AbsoluteEncoderConfigBuilder offset(Rotation2d offset) {
      this.offset = offset;
      return this;
    }

    public AbsoluteEncoderConfig build() {
      return new AbsoluteEncoderConfig(ccwPositive, sensorToMechanismRatio, offset);
    }
  }

  /**
   * Creates Phoenix 6 magnet sensor configs.
   *
   * @return Phoenix 6 magnet sensor configs.
   */
  private MagnetSensorConfigs createMagnetSensorConfigs() {
    final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

    magnetSensorConfigs.MagnetOffset = offset().getRotations();
    magnetSensorConfigs.SensorDirection =
        ccwPositive()
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;

    return magnetSensorConfigs;
  }

  /**
   * Creates a Phoenix 6 CANcoder config.
   *
   * @return a Phoenix 6 CANcoder config.
   */
  public CANcoderConfiguration createCANcoderConfig() {
    final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    cancoderConfig.MagnetSensor = createMagnetSensorConfigs();

    return cancoderConfig;
  }
}
