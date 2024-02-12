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

  /** Constants for the hardware configuration of the robot. */
  public static class HardwareConstants {
    /** If true, use real arm hardware. */
    public static final boolean REAL_ARM = false;

    /** If true, use real climber hardware. */
    public static final boolean REAL_CLIMBER = false;

    /** If true, use real intake hardware. */
    public static final boolean REAL_INTAKE = true;

    /** If true, use real lights hardware. */
    public static final boolean REAL_LIGHTS = true;

    /** If true, use real odometry hardware. */
    public static final boolean REAL_ODOMETRY = true;

    /** If true, use real shooter hardware. */
    public static final boolean REAL_SHOOTER = false;

    /** If true, use real swerve hardware. */
    public static final boolean REAL_SWERVE = true;

    /** If true, use real vision hardware. */
    public static final boolean REAL_VISION = true;
  }
}
