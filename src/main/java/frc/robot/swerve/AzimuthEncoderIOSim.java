package frc.robot.swerve;

/** Simulated azimuth encoder. */
public class AzimuthEncoderIOSim implements AzimuthEncoderIO {

    @Override
    public void update(AzimuthEncoderIOValues values) {
        values.angleRotations = 0.0;
    }
    
}
