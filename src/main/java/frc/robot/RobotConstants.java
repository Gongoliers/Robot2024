package frc.robot;

/** Constants for the entire robot. */
public class RobotConstants {

  /** Number of robot periodic calls per second. */
  public static final double PERIODIC_RATE = 50;

  /** Duration of each robot periodic call in seconds. */
  public static final double PERIODIC_DURATION = 1.0 / PERIODIC_RATE;

  /** Voltage of the robot's battery, */
  public static final double BATTERY_VOLTAGE = 12.0;

  /**
   * Duration between the robot being disabled and the swerve subsystem is allowed to coast in
   * seconds.
   */
  public static final double DISABLE_COAST_DELAY = 3.0;
}
