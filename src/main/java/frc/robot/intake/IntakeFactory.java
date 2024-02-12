package frc.robot.intake;

import frc.robot.Robot;
import frc.robot.RobotConstants.HardwareConstants;

/** Helper class for creating hardware for the intake subsystem. */
public class IntakeFactory {

  /**
   * Creates a roller motor.
   *
   * @return a roller motor.
   */
  public static RollerMotorIO createRollerMotor() {
    if (Robot.isReal() && HardwareConstants.REAL_INTAKE) return new RollerMotorIOSparkMax();

    return new RollerMotorIOSim();
  }
}
