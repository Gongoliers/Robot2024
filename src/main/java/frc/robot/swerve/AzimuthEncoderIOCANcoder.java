package frc.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.CAN;
import frc.lib.ConfigApplier;

/** CANcoder azimuth encoder. */
public class AzimuthEncoderIOCANcoder implements AzimuthEncoderIO {

  /** Hardware CANcoder. */
  private final CANcoder cancoder;

  /** CANcoder's absolute position status signal. */
  private final StatusSignal<Double> absolutePositionRotations;

  /** CANcoder's magnet offset. */
  private final Rotation2d magnetOffset;

  /**
   * Creates a new CANcoder azimuth encoder.
   *
   * @param azimuthEncoderCAN the CANcoder's CAN identifier.
   * @param magnetOffset the CANcoder's magnet offset.
   */
  public AzimuthEncoderIOCANcoder(CAN azimuthEncoderCAN, Rotation2d magnetOffset) {
    cancoder = new CANcoder(azimuthEncoderCAN.id(), azimuthEncoderCAN.bus());

    absolutePositionRotations = cancoder.getAbsolutePosition();

    this.magnetOffset = magnetOffset;
  }

  @Override
  public void configure() {
    CANcoderConfiguration config = SwerveFactory.createAzimuthEncoderConfig();

    config.MagnetSensor.MagnetOffset = magnetOffset.getRotations();

    ConfigApplier.applyCANcoderConfig(cancoder.getConfigurator(), config);
  }

  @Override
  public void update(AzimuthEncoderIOValues values) {
    absolutePositionRotations.refresh();

    values.positionRotations = absolutePositionRotations.getValue();
  }
}
