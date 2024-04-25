package frc.robot.swerve;

import frc.lib.controller.PositionControllerIO;
import frc.lib.controller.PositionControllerIOSim;
import frc.lib.controller.PositionControllerIOTalonFXSteer;
import frc.lib.controller.VelocityControllerIO;
import frc.lib.controller.VelocityControllerIOSim;
import frc.lib.controller.VelocityControllerIOTalonFXPIDF;
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
   * Creates a steer motor.
   *
   * @return a steer motor.
   */
  public static PositionControllerIO createSteerMotor(SwerveModuleConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SWERVE))
      return new PositionControllerIOTalonFXSteer(
          config.moduleCAN().steer(),
          config.moduleCAN().azimuth(),
          SwerveConstants.STEER_CONFIG.withAbsoluteEncoderConfig(
              SwerveConstants.STEER_CONFIG.absoluteEncoderConfig().withOffset(config.offset())),
          false);

    return new PositionControllerIOSim();
  }

  /**
   * Creates a drive motor.
   *
   * @return a drive motor.
   */
  public static VelocityControllerIO createDriveMotor(SwerveModuleConfig config) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SWERVE))
      return new VelocityControllerIOTalonFXPIDF(
          config.moduleCAN().drive(), SwerveConstants.DRIVE_CONFIG, false);

    return new VelocityControllerIOSim();
  }
}
