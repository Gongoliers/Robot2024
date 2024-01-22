package frc.robot.intake;

import frc.robot.Robot;

/** Helper class for creating hardware for the intake subsystem. */
public class IntakeFactory {

  /**
   * Creates an intake motor.
   *
   * @return an intake motor.
   */
  public static IntakeMotorIO createIntakeMotor() {
    if (Robot.isReal()) return new IntakeMotorIOSparkMax();

    return new IntakeMotorIOSim();
  }
}
