package frc.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Swerve module hardware interface. */
public interface SwerveModuleIO {

  /**
   * Gets the state of the swerve module.
   *
   * @return the state of the swerve module.
   */
  public SwerveModuleState getState();

  /**
   * Gets the setpoint of the swerve module.
   *
   * @return the setpoint of the swerve module.
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
   * Gets the position of the swerve module.
   *
   * @return the position of the swerve module.
   */
  public SwerveModulePosition getPosition();

  /**
   * Sets the swerve module's brake mode.
   *
   * @param brake if true, brake.
   */
  public void setBrake(boolean brake);
}
