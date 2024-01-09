package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Constants for the swerve subsystem. */
public class SwerveConstants {

    /** Constants for the MK4i COTS module. */
    public static class MK4iConstants {
        /** Gearing between the steer motor and the wheel. */
        public static final double STEER_GEARING = 150.0 / 7.0; 

        /** Moment of inertia of the wheel when steering in joules kilograms meters squared. */
        public static final double STEER_MOI = 0.004; // TODO
    }

    /** Module configuration for the north west swerve module. */
    public static final SwerveModuleConfig NORTH_WEST_MODULE_CONFIG = new SwerveModuleConfig(new SwerveModuleCAN(0, 0, 0, ""), new Translation2d(), new Rotation2d()); // TODO

    /** Module configuration for the north east swerve module. */
    public static final SwerveModuleConfig NORTH_EAST_MODULE_CONFIG = new SwerveModuleConfig(new SwerveModuleCAN(0, 0, 0, ""), new Translation2d(), new Rotation2d()); // TODO

    /** Module configuration for the south east swerve module. */
    public static final SwerveModuleConfig SOUTH_EAST_MODULE_CONFIG = new SwerveModuleConfig(new SwerveModuleCAN(0, 0, 0, ""), new Translation2d(), new Rotation2d()); // TODO

    /** Module configuration for the south west swerve module. */
    public static final SwerveModuleConfig SOUTH_WEST_MODULE_CONFIG = new SwerveModuleConfig(new SwerveModuleCAN(0, 0, 0, ""), new Translation2d(), new Rotation2d()); // TODO

    /** Maximum attainable speed in meters per second. */
    public static final double MAXIMUM_SPEED = Units.feetToMeters(4.0);
}
