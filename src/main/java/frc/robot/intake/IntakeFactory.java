package frc.robot.intake;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the intake subsystem. */
public class IntakeFactory {

  /**
   * Creates a roller motor.
   *
   * @return a roller motor.
   */
  public static RollerMotorIO createRollerMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE))
      return new RollerMotorIOTalonFX();

    return new RollerMotorIOSim();
  }
}
