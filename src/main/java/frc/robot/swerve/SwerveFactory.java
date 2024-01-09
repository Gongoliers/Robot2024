package frc.robot.swerve;

import frc.robot.Robot;

/** Helper class for creating hardware for the swerve subsystem. */
public class SwerveFactory {

    /**
     * Creates a swerve module.
     * 
     * @return a swerve module.
     */
    public static SwerveModuleIO createModule(SwerveModuleConfig config) {
        if (Robot.isReal()) return new SwerveModuleIOCustom(); // TODO

        return new SwerveModuleIOCustom();
    }

    /**
     * Creates an azimuth encoder.
     * 
     * @return an azimuth encoder.
     */
    public static AzimuthEncoderIO createAzimuthEncoder() {
        if (Robot.isReal()) return new AzimuthEncoderIOSim();

        return new AzimuthEncoderIOSim();
    }

    /**
     * Creates a steer motor.
     * 
     * @return a steer motor.
     */
    public static SteerMotorIO createSteerMotor() {
        if (Robot.isReal()) return new SteerMotorIOSim();

        return new SteerMotorIOSim();
    }

    /**
     * Creates a drive motor.
     * 
     * @return a drive motor.
     */
    public static DriveMotorIO createDriveMotor() {
        if (Robot.isReal()) return new DriveMotorIOSim();

        return new DriveMotorIOSim();
    }

}
