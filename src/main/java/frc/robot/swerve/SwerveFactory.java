package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the swerve subsystem. */
public class SwerveFactory {

  /**
   * Creates a swerve module.
   *
   * @return a swerve module.
   */
  public static SwerveModuleIO createModule(SwerveModuleConfig config) {
    return new SwerveModuleIOCustom(config);
  }

  /**
   * Creates an azimuth encoder.
   *
   * @return an azimuth encoder.
   */
  public static AzimuthEncoderIO createAzimuthEncoder(SwerveModuleConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SWERVE))
      return new AzimuthEncoderIOCANcoder(config.moduleCAN().azimuth(), config.offset());

    return new AzimuthEncoderIOSim();
  }

  /**
   * Creates an azimuth encoder configuration.
   *
   * @return an azimuth encoder configuration.
   */
  public static CANcoderConfiguration createAzimuthEncoderConfig() {
    CANcoderConfiguration azimuthEncoderConfig = new CANcoderConfiguration();

    return azimuthEncoderConfig;
  }

  /**
   * Creates a steer motor.
   *
   * @return a steer motor.
   */
  public static SteerMotorIO createSteerMotor(SwerveModuleConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SWERVE))
      return new SteerMotorIOTalonFXPIDF(config.moduleCAN().steer(), config.moduleCAN().azimuth());

    return new SteerMotorIOSim();
  }

  /**
   * Creates a drive motor.
   *
   * @return a drive motor.
   */
  public static DriveMotorIO createDriveMotor(SwerveModuleConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SWERVE))
      return new DriveMotorIOTalonFXPID(config.moduleCAN().drive());

    return new DriveMotorIOSim();
  }
}
