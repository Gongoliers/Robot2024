package frc.lib.config;

/** Motor config. */
public class MotorConfig {

    /** Use the motor to brake the controller mechanism on neutral output. */
    private boolean neutralBrake = false;

    /** Interpret counterclockwise rotation on the motor face as having positive velocity. */
    private boolean ccwPositive = true;

    /** Maximum amount of current, in amps, to provide to the motor. */
    private double currentLimitAmps = 40.0;

    /**
     * Modifies this motor config's neutral brake.
     * 
     * @param neutralBrake the neutral brake.
     * @return this motor config.
     */
    public MotorConfig withNeutralBrake(boolean neutralBrake) {
        this.neutralBrake = neutralBrake;
        return this;
    }

    /**
     * Modifies this motor config's counterclockwise positive.
     * 
     * @param ccwPositive the counterclockwise positive.
     * @return this motor config.
     */
    public MotorConfig withCCWPositive(boolean ccwPositive) {
        this.ccwPositive = ccwPositive;
        return this;
    }

    /**
     * Modifies this motor config's current limit.
     * 
     * @param currentLimitAmps the current limit.
     * @return this motor config.
     */
    public MotorConfig withCurrentLimit(double currentLimitAmps) {
        this.currentLimitAmps = currentLimitAmps;
        return this;
    }

    /**
     * Returns true if motor should neutral brake.
     * 
     * @return true if motor should neutral brake.
     */
    public boolean neutralBrake() {
        return neutralBrake;
    }

    /**
     * Returns true if the motor is counterclockwise positive.
     * 
     * @return true if the motor is counterclockwise positive.
     */
    public boolean ccwPositive() {
        return ccwPositive;
    }

    /**
     * Returns the motor current limit, in amps.
     * 
     * @return the motor current limit, in amps.
     */
    public double currentLimitAmps() {
        return currentLimitAmps;
    }
}
