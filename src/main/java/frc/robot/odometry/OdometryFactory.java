package frc.robot.odometry;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.lib.CAN;
import frc.robot.Robot;

/** Helper class for creating hardware for the odometry subsystem. */
public class OdometryFactory {

  /**
   * Creates a gyroscope.
   *
   * @return a gyroscope.
   */
  public static GyroscopeIO createGyroscope(CAN gyroscopeCAN, Odometry odometry) {
    //if (Robot.isReal()) return new GyroscopeIOPigeon2(gyroscopeCAN);

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
