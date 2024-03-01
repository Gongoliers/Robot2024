package frc.robot.arm;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the arm subsystem. */
public class ArmFactory {

  public static LimitSwitchIO createLimitSwitch() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.ARM))
      return new LimitSwitchIODigital();

    return new LimitSwitchIOSim();
  }

  /**
   * Creates a shoulder motor.
   *
   * @return a shoulder motor.
   */
  public static ShoulderMotorIO createShoulderMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.ARM))
      return new ShoulderMotorIOSparkMax();

    return new ShoulderMotorIOSim();
  }

  /**
   * Creates an elbow motor.
   *
   * @return an elbow motor.
   */
  public static WristMotorIO createWristMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.ARM))
      return new WristMotorIOSparkMax();

    return new WristMotorIOSim();
  }
}
