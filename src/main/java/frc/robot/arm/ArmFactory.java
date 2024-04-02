package frc.robot.arm;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the arm subsystem. */
public class ArmFactory {

  /**
   * Creates a shoulder motor.
   *
   * @return a shoulder motor.
   */
  public static ShoulderMotorIO createShoulderMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.ARM))
      return new ShoulderMotorIOTalonFX();

    return new ShoulderMotorIOSim();
  }
}
