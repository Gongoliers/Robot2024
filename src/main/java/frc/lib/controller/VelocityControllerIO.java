package frc.lib.controller;

/** Velocity controller interface. */
public interface VelocityControllerIO {

    /** Velocity controller constants. */
    public static class VelocityControllerIOConstants {
        /** Interpret counterclockwise rotation on the motor face as having positive velocity, if set to true. */
        public boolean ccwPositive = true;

        /** Use the motor to brake the controlled mechanism on neutral output, if set to true. */
        public boolean neutralBrake = false;

        /** Maximum amount of current, in amps, to provide to the motor. */
        public double currentLimitAmps = 40.0;

        /** Ratio between the velocity sensor and the controlled mechanism. */
        public double sensorToMechanismRatio = 1.0;
    }

    /** Velocity controller values. */
    public static class VelocityControllerIOValues {
        /** Velocity in rotations per second. */
        public double velocityRotationsPerSecond = 0.0;

        /** Acceleration in rotations per second per second. */
        public double accelerationRotationsPerSecondPerSecond = 0.0;

        /** Motor voltage in volts. */
        public double motorVolts = 0.0;

        /** Motor current in amps. */
        public double motorAmps = 0.0;
    }

    /**
     * Configures the velocity controller. 
     * 
     * @param constants
    */
    public void configure(VelocityControllerIOConstants constants);
    
    /**
     * Updates the velocity controller's values. 
     * 
     * @param values
    */
    public void update(VelocityControllerIOValues values);

    /**
     * Sets the velocity setpoint.
     * 
     * @param velocityRotationsPerSecond
     */
    public void setSetpoint(double velocityRotationsPerSecond);

}
