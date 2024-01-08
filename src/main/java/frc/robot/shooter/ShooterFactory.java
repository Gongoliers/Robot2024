package frc.robot.shooter;

import frc.robot.Robot;

/** Helper class for creating hardware for the shooter subsystem. */
public class ShooterFactory {

  /**
   * Creates a beam break sensor.
   *
   * @return a beam break sensor.
   */
  public static BeamBreakSensorIO createBeamBreakSensor() {
    if (Robot.isReal()) return new BeamBreakSensorIOSim(); // TODO

    return new BeamBreakSensorIOSim();
  }

  /**
   * Creates a serializer motor.
   *
   * @return a serializer motor.
   */
  public static SerializerMotorIO createSerializerMotor() {
    if (Robot.isReal()) return new SerializerMotorIOSim(); // TODO

    return new SerializerMotorIOSim();
  }

  /**
   * Creates a flywheel motor.
   *
   * @return a flywheel motor.
   */
  public static FlywheelMotorIO createFlywheelMotor() {
    if (Robot.isReal()) return new FlywheelMotorIOSim(); // TODO

    return new FlywheelMotorIOSim();
  }
}
