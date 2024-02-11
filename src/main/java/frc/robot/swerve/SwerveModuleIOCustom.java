package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;
import frc.robot.swerve.SteerMotorIO.SteerMotorIOValues;

/** Custom swerve module. */
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

  /** Module setpoint */
  private SwerveModuleState setpoint;

  /**
   * Creates a custom swerve module.
   *
   * @param config the swerve module's configuration.
   */
  public SwerveModuleIOCustom(SwerveModuleConfig config) {
    azimuthEncoder = SwerveFactory.createAzimuthEncoder(config);
    azimuthEncoder.configure();

    steerMotor = SwerveFactory.createSteerMotor(config);
    steerMotor.configure();

    driveMotor = SwerveFactory.createDriveMotor(config);
    driveMotor.configure();

    setpoint = new SwerveModuleState();

    azimuthEncoder.update(azimuthEncoderValues);
    steerMotor.setPosition(azimuthEncoderValues.positionRotations);
  }

  @Override
  public SwerveModuleState getState() {
    steerMotor.update(steerMotorValues);
    driveMotor.update(driveMotorValues);

    return new SwerveModuleState(
        driveMotorValues.velocityMetersPerSecond,
        Rotation2d.fromRotations(steerMotorValues.positionRotations));
  }

  @Override
  public Rotation2d getAzimuth() {
    azimuthEncoder.update(azimuthEncoderValues);

    return Rotation2d.fromRotations(azimuthEncoderValues.positionRotations);
  }

  @Override
  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  @Override
  public void runSetpoint(SwerveModuleState setpoint, boolean lazy) {
    if (lazy) {
      setpoint = optimize(setpoint, getState());
    }

    steerMotor.runSetpoint(setpoint.angle.getRotations());
    driveMotor.runSetpoint(setpoint.speedMetersPerSecond);

    this.setpoint = setpoint;
  }

  /**
   * Optimizes a swerve module's setpoint.
   *
   * @param setpoint the setpoint to optimize.
   * @param state the state of the module.
   * @return the optimized setpoint.
   */
  private SwerveModuleState optimize(SwerveModuleState setpoint, SwerveModuleState state) {
    setpoint = SwerveModuleState.optimize(setpoint, state.angle);

    if (setpoint.speedMetersPerSecond == 0.0) {
      setpoint.angle = state.angle;
    } else {
      Rotation2d angleError = setpoint.angle.minus(state.angle);

      setpoint.speedMetersPerSecond *= angleError.getCos();
    }

    return setpoint;
  }

  @Override
  public SwerveModulePosition getPosition() {
    azimuthEncoder.update(azimuthEncoderValues);
    steerMotor.update(steerMotorValues);
    driveMotor.update(driveMotorValues);

    return new SwerveModulePosition(
        driveMotorValues.positionMeters,
        Rotation2d.fromRotations(steerMotorValues.positionRotations));
  }

  @Override
  public void setBrake(boolean brake) {
    driveMotor.setBrake(brake);
  }
}
