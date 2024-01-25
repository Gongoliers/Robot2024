package frc.robot.intake;

import frc.robot.Robot;

/** Helper class for creating hardware for the intake subsystem. */
public class IntakeFactory {

  /**
   * Creates a roller motor.
   *
   * @return a roller motor.
   */
  public static RollerMotorIO createRollerMotor() {
    if (Robot.isReal()) return new RollerMotorIOSparkMax();

    return new RollerMotorIOSim();
  }
}
