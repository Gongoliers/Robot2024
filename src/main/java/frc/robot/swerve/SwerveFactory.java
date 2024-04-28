package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

/** Factory for creating swerve subsystem hardware. */
public class SwerveFactory {

  /**
   * Creates a swerve module.
   *
   * @return a swerve module.
   */
  private static SwerveModuleIO createModule(CAN steer, CAN azimuth, CAN drive, Rotation2d offset) {
    return new SwerveModuleIOCustom(steer, azimuth, drive, offset);
  }

  /**
   * creates the north west swerve module.
   *
   * @return the north west swerve module.
   */
  public static SwerveModuleIO createNorthWestModule() {
    return createModule(
        new CAN(8, "swerve"),
        new CAN(16, "swerve"),
        new CAN(24, "swerve"),
        Rotation2d.fromRotations(-0.084717).unaryMinus());
  }

  /**
   * Creates the north west swerve module translation.
   *
   * @return the north west swerve module translation.
   */
  public static Translation2d createNorthWestModuleTranslation() {
    return new Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(10.375));
  }

  /**
   * creates the north east swerve module.
   *
   * @return the north east swerve module.
   */
  public static SwerveModuleIO createNorthEastModule() {
    return createModule(
        new CAN(16, "swerve"),
        new CAN(18, "swerve"),
        new CAN(30, "swerve"),
        Rotation2d.fromRotations(0.196777).unaryMinus());
  }

  /**
   * Creates the north east swerve module translation.
   *
   * @return the north east swerve module translation.
   */
  public static Translation2d createNorthEastModuleTranslation() {
    return new Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(-10.375));
  }

  /**
   * creates the south east swerve module.
   *
   * @return the south east swerve module.
   */
  public static SwerveModuleIO createSouthEastModule() {
    return createModule(
        new CAN(12, "swerve"),
        new CAN(22, "swerve"),
        new CAN(26, "swerve"),
        Rotation2d.fromRotations(0.276611).unaryMinus());
  }

  /**
   * Creates the south east swerve module translation.
   *
   * @return the south east swerve module translation.
   */
  public static Translation2d createSouthEastModuleTranslation() {
    return new Translation2d(Units.inchesToMeters(-10.375), Units.inchesToMeters(-10.375));
  }

  /**
   * creates the south west swerve module.
   *
   * @return the south west swerve module.
   */
  public static SwerveModuleIO createSouthWestModule() {
    return createModule(
        new CAN(10, "swerve"),
        new CAN(20, "swerve"),
        new CAN(28, "drive"),
        Rotation2d.fromRotations(0.223145).unaryMinus());
  }

  /**
   * Creates the south west swerve module translation.
   *
   * @return the south west swerve module translation.
   */
  public static Translation2d createSouthWestModuleTranslation() {
    return new Translation2d(Units.inchesToMeters(-10.375), Units.inchesToMeters(10.375));
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
      return new VelocityControllerIOTalonFXPIDF(drive, SwerveConstants.DRIVE_CONFIG, false);

    return new VelocityControllerIOSim();
  }
}
