package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.CAN;
import frc.lib.config.AbsoluteEncoderConfig;
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
  public static PositionControllerIO createSteerMotor(CAN steer, CAN azimuth, Rotation2d offset) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SWERVE))
      return new PositionControllerIOTalonFXSteer(
          steer,
          azimuth,
          SwerveConstants.STEER_CONFIG.withAbsoluteEncoderConfig(
              new AbsoluteEncoderConfig().withOffset(offset)),
          false);

    return new PositionControllerIOSim();
  }

  /**
   * Creates a drive motor.
   *
   * @return a drive motor.
   */
  public static VelocityControllerIO createDriveMotor(CAN drive) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SWERVE))
      return new VelocityControllerIOTalonFXPIDF(
          drive, SwerveConstants.DRIVE_CONFIG, false);

    return new VelocityControllerIOSim();
  }
}
