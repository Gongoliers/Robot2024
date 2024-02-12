package frc.robot.shooter;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the shooter subsystem. */
public class ShooterFactory {

  /**
   * Creates a beam break sensor.
   *
   * @return a beam break sensor.
   */
  public static BeamBreakSensorIO createBeamBreakSensor() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.SHOOTER))
      return new BeamBreakSensorIOSim(); // TODO

    return new BeamBreakSensorIOSim();
  }

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
      return new FlywheelMotorIOSparkMax(); // TODO

    return new FlywheelMotorIOSim();
  }
}
