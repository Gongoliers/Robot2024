package frc.robot.shooter;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the shooter subsystem. */
public class ShooterFactory {

  /**
   * Creates a serializer motor.
   *
   * @return a serializer motor.
   */
  public static SerializerMotorIO createSerializerMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER))
      return new SerializerMotorIOSparkMax();

    return new SerializerMotorIOSim();
  }

  /**
   * Creates a flywheel motor.
   *
   * @return a flywheel motor.
   */
  public static FlywheelMotorIO createFlywheelMotor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER))
      return new FlywheelMotorIOTalonFX();

    return new FlywheelMotorIOSim();
  }
}
