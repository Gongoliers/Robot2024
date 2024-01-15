package frc.robot.swerve;

/** Simulated azimuth encoder. */
public class AzimuthEncoderIOSim implements AzimuthEncoderIO {

  @Override
  public void configure() {}

  @Override
  public void update(AzimuthEncoderIOValues values) {
    values.positionRotations = 0.0;
  }
}
