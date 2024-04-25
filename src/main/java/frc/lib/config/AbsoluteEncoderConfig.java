package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;

/** Absolute encoder config. */
public class AbsoluteEncoderConfig {

    /** Interpret counterclockwise rotation on the encoder as having positive velocity. */
    private boolean ccwPositive = true;

    /** Ratio between the absolute encoder and the mechanism. */
    private double sensorToMechanismRatio = 1.0;

    /** Offset between absolute encoder reading and mechanism position. */
    private Rotation2d offset = new Rotation2d();

    /**
     * Modifies this absolute encoder config's counterclockwise positive.
     * 
     * @param ccwPositive the counterclockwise positive.
     * @return this absolute encoder config.
     */
    public AbsoluteEncoderConfig withCCWPositive(boolean ccwPositive) {
        this.ccwPositive = ccwPositive;
        return this;
    }

    /**
     * Modifies this absolute encoder config's sensor to mechanism ratio.
     * 
     * @param sensorToMechanismRatio the sensor to mechanism ratio.
     * @return this absolute encoder config.
     */
    public AbsoluteEncoderConfig withSensorToMechanismRatio(double sensorToMechanismRatio) {
        this.sensorToMechanismRatio = sensorToMechanismRatio;
        return this;
    }

    /**
     * Modifies this absolute encoder config's offset.
     * 
     * @param offset the offset.
     * @return this absolute encoder config.
     */
    public AbsoluteEncoderConfig withOffset(Rotation2d offset) {
        this.offset = offset;
        return this;
    }

    /**
     * Returns true if the absolute encoder is counterclockwise positive.
     * 
     * @return true if the absolute encoder is counterclockwise positive.
     */
    public boolean ccwPositive() {
        return ccwPositive;
    }

    /**
     * Returns the absolute encoder sensor to mechanism ratio.
     * 
     * @return the absolute encoder sensor to mechanism ratio.
     */
    public double sensorToMechanismRatio() {
        return sensorToMechanismRatio;
    }

    /**
     * Returns the absolute encoder offset.
     * 
     * @return the absolute encoder offset.
     */
    public Rotation2d offset() {
        return offset;
    }
}
