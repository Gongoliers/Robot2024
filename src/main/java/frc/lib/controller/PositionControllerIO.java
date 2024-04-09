package frc.lib.controller;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Telemetry;

/** Position controller interface. */
public interface PositionControllerIO {

    /** Position controller constants. */
    public static class PositionControllerIOConstants {
        /** Interpret counterclockwise rotation on the motor face as having positive velocity, if set to true. */
        public boolean ccwPositive = true;

        /** Use the motor to brake the controlled mechanism on neutral output, if set to true. */
        public boolean neutralBrake = false;

        /** Maximum amount of current, in amps, to provide to the motor. */
        public double currentLimitAmps = 40.0;

        /** Ratio between the position sensor and the controlled mechanism. */
        public double sensorToMechanismRatio = 1.0;

        /** Offset between absolute encoder reading and controlled mechanism position in rotations. */
        public double absoluteEncoderOffsetRotations = 0.0;
    }

    /** Position controller values. */
    public static class PositionControllerIOValues {
        /** Position in rotations. */
        public double positionRotations = 0.0;

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
     * Adds position controller values to Shuffleboard.
     * 
     * @param tab
     * @param name
     * @param values
     */
    public static void addToShuffleboard(ShuffleboardTab tab, String name, PositionControllerIOValues values) {
        ShuffleboardLayout positionController = Telemetry.addColumn(tab, name);

        positionController.addDouble("Position (rot)", () -> values.positionRotations);
        positionController.addDouble("Velocity (rps)", () -> values.velocityRotationsPerSecond);
        positionController.addDouble("Acceleration (rpsps)", () -> values.accelerationRotationsPerSecondPerSecond);
        positionController.addDouble("Voltage (V)", () -> values.motorVolts);
        positionController.addDouble("Current (A)", () -> values.motorAmps);
    }
    
    /**
     * Configures the position controller.
     * 
     * @param constants
     */
    public void configure(PositionControllerIOConstants constants);

    /**
     * Updates the position controller's values.
     * 
     * @param values
     */
    public void update(PositionControllerIOValues values);

    /**
     * Sets the mechanism position.
     * 
     * @param positionRotations
     */
    public void setPosition(double positionRotations);

    /**
     * Sets the position setpoint.
     * 
     * @param positionRotations
     * @param velocityRotationsPerSecond
     */
    public void setSetpoint(double positionRotations, double velocityRotationsPerSecond);
}
