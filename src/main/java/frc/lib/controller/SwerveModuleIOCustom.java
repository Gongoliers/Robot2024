package frc.lib.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.controller.PositionControllerIO.PositionControllerIOValues;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOValues;

/** Custom swerve module. */
public class SwerveModuleIOCustom implements SwerveModuleIO {

  /** Steer motor. */
  private final PositionControllerIO steerMotor;

  /** Steer motor values. */
  private final PositionControllerIOValues steerMotorValues = new PositionControllerIOValues();

  /** Drive motor. */
  private final VelocityControllerIO driveMotor;

  /** Drive motor values. */
  private final VelocityControllerIOValues driveMotorValues = new VelocityControllerIOValues();

  /** Wheel circumference. */
  private final double wheelCircumference;

  /** Module setpoint */
  private SwerveModuleState setpoint;

  public SwerveModuleIOCustom(
      PositionControllerIO steerMotor, VelocityControllerIO driveMotor, double wheelCircumference) {
    this.steerMotor = steerMotor;
    this.steerMotor.configure();

    this.driveMotor = driveMotor;
    this.driveMotor.configure();

    this.wheelCircumference = wheelCircumference;

    setpoint = new SwerveModuleState();
  }

  @Override
  public SwerveModuleState getState() {
    steerMotor.update(steerMotorValues);
    driveMotor.update(driveMotorValues);

    return new SwerveModuleState(
        driveMotorValues.velocityRotationsPerSecond * wheelCircumference,
        Rotation2d.fromRotations(steerMotorValues.positionRotations));
  }

  @Override
  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  @Override
  public void setSetpoint(SwerveModuleState setpoint, boolean lazy) {
    setpoint = optimize(setpoint, getState(), lazy);

    steerMotor.setSetpoint(setpoint.angle.getRotations(), 0);
    driveMotor.setSetpoint(setpoint.speedMetersPerSecond / wheelCircumference);

    this.setpoint = setpoint;
  }

  /**
   * Optimizes a swerve module setpoint.
   *
   * @param setpoint the setpoint to optimize.
   * @param state the state of the module.
   * @param lazy if true, perform additional optimizations on the setpoint.
   * @return the optimized setpoint.
   */
  private SwerveModuleState optimize(
      SwerveModuleState setpoint, SwerveModuleState state, boolean lazy) {
    // Always perform this optimization, even when lazy
    setpoint = SwerveModuleState.optimize(setpoint, state.angle);

    // If we aren't lazy, don't perform additional optimizations
    if (!lazy) {
      return setpoint;
    }

    // Since we are lazy, perform additional optimizations

    // Deadband the module speed
    if (Math.abs(setpoint.speedMetersPerSecond) < Units.inchesToMeters(1)) {
      setpoint.speedMetersPerSecond = 0.0;
    }

    // Keep previous angle if the module isn't moving
    if (setpoint.speedMetersPerSecond == 0.0) {
      setpoint.angle = state.angle;
    }

    // Scale our speed by the module's error
    Rotation2d error = setpoint.angle.minus(state.angle);
    setpoint.speedMetersPerSecond *= error.getCos();

    return setpoint;
  }

  @Override
  public SwerveModulePosition getPosition() {
    steerMotor.update(steerMotorValues);
    driveMotor.update(driveMotorValues);

    return new SwerveModulePosition(
        driveMotorValues.positionRotations * wheelCircumference,
        Rotation2d.fromRotations(steerMotorValues.positionRotations));
  }
}
