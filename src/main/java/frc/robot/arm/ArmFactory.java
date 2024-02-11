package frc.robot.arm;

import frc.robot.Robot;

/** Helper class for creating hardware for the arm subsystem. */
public class ArmFactory {

  /**
   * Creates a shoulder motor.
   *
   * @return a shoulder motor.
   */
  public static ShoulderMotorIO createShoulderMotor() {
    // if (Robot.isReal()) return new ShoulderMotorIOSparkMax();

    return new ShoulderMotorIOSim();
  }

  /**
   * Creates an elbow motor.
   *
   * @return an elbow motor.
   */
  public static ElbowMotorIO createElbowMotor() {
    return new ElbowMotorIOSim();
  }
}
