package frc.robot.intake;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the intake subsystem. */
public class IntakeFactory {

  /**
   * Creates a pivot motor.
   *
   * @return a pivot motor.
   */
  public static PivotMotorIO createPivotMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE))
      return new PivotMotorIOTalonSRX();

    return new PivotMotorIOSim();
  }

  /**
   * Creates a roller motor.
   *
   * @return a roller motor.
   */
  public static RollerMotorIO createRollerMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.INTAKE))
      return new RollerMotorIOSparkMax();

    return new RollerMotorIOSim();
  }
}
