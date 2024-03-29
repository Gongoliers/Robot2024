package frc.robot.odometry;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the odometry subsystem. */
public class OdometryFactory {

  /**
   * Creates a gyroscope.
   *
   * @return a gyroscope.
   */
  public static GyroscopeIO createGyroscope(Odometry odometry) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.ODOMETRY))
      return new GyroscopeIOPigeon2();

    return new GyroscopeIOSim(odometry);
  }

  /**
   * Creates a gyroscope configuration.
   *
   * @return a gyroscope configuration.
   */
  public static Pigeon2Configuration createGyroscopeConfig() {
    Pigeon2Configuration gyroscopeConfig = new Pigeon2Configuration();

    gyroscopeConfig.Pigeon2Features.EnableCompass = false;

    return gyroscopeConfig;
  }
}
