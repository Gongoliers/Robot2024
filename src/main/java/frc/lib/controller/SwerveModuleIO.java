package frc.lib.controller;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Swerve module interface. */
public interface SwerveModuleIO {

  /**
   * Returns the swerve module state.
   *
   * @return the swerve module state.
   */
  public SwerveModuleState getState();

  /**
   * Returns the swerve module setpoint.
   *
   * @return the swerve module setpoint.
   */
  public SwerveModuleState getSetpoint();

  /**
   * Sets the swerve module setpoint.
   *
   * @param setpoint the swerve module setpoint.
   * @param lazy if true, optimize the swerve module setpoint.
   */
  public void setSetpoint(SwerveModuleState setpoint, boolean lazy);

  /**
   * Returns the swerve module position.
   *
   * @return the swerve module position.
   */
  public SwerveModulePosition getPosition();
}
