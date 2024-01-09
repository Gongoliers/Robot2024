package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;
import frc.robot.swerve.SteerMotorIO.SteerMotorIOValues;

public class SwerveModuleIOCustom implements SwerveModuleIO {

  /** Azimuth encoder. */
  private final AzimuthEncoderIO azimuthEncoder;

  /** Azimuth encoder values. */
  private final AzimuthEncoderIOValues azimuthEncoderValues = new AzimuthEncoderIOValues();

  /** Steer motor. */
  private final SteerMotorIO steerMotor;

  /** Steer motor values. */
  private final SteerMotorIOValues steerMotorValues = new SteerMotorIOValues();

  /** Drive motor. */
  private final DriveMotorIO driveMotor;

  /** Driver motor values. */
  private final DriveMotorIOValues driveMotorValues = new DriveMotorIOValues();

  public SwerveModuleIOCustom() {
    azimuthEncoder = SwerveFactory.createAzimuthEncoder();
    steerMotor = SwerveFactory.createSteerMotor();
    driveMotor = SwerveFactory.createDriveMotor();

    azimuthEncoder.update(azimuthEncoderValues);
    steerMotor.setPosition(azimuthEncoderValues.angleRotations);
  }

  @Override
  public void setSetpoint(SwerveModuleState setpoint, boolean lazy) {
    if (lazy) {
      setpoint = optimize(setpoint);
    }

    steerMotor.setSetpoint(setpoint.angle.getRotations());
    driveMotor.setSetpoint(setpoint.speedMetersPerSecond);
  }

  /**
   * Optimizes a swerve module's setpoint.
   *
   * @param setpoint the setpoint to optimize.
   * @return the optimized setpoint.
   */
  private SwerveModuleState optimize(SwerveModuleState setpoint) {
    return setpoint; // TODO
  }

  @Override
  public SwerveModuleState getState() {
    azimuthEncoder.update(azimuthEncoderValues);
    steerMotor.update(steerMotorValues);
    driveMotor.update(driveMotorValues);

    return new SwerveModuleState(
        driveMotorValues.velocityMetersPerSecond,
        Rotation2d.fromRotations(steerMotorValues.angleRotations));
  }

  @Override
  public SwerveModulePosition getPosition() {
    azimuthEncoder.update(azimuthEncoderValues);
    steerMotor.update(steerMotorValues);
    driveMotor.update(driveMotorValues);

    return new SwerveModulePosition(
        driveMotorValues.positionMeters, Rotation2d.fromRotations(steerMotorValues.angleRotations));
  }
}
