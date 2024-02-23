package frc.robot;

import edu.wpi.first.math.util.Units;
import java.util.EnumSet;
import java.util.Set;

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

  /** Distance from the frame perimeter to the origin in meters. */
  public static final double FRAME_PERIMETER_TO_ORIGIN_DISTANCE = Units.inchesToMeters(14);

  /** Maximum horizontal extension distance in meters. */
  public static final double MAX_HORIZONTAL_EXTENSION_DISTANCE = Units.feetToMeters(1);

  /** Maximum vertical extension distance in meters. */
  public static final double MAX_VERTICAL_EXTENSION_DISTANCE = Units.inchesToMeters(48);

  /** Robot subsystems. */
  public enum Subsystem {
    ARM,
    CLIMBER,
    INTAKE,
    LIGHTS,
    ODOMETRY,
    SHOOTER,
    SWERVE,
    VISION
  }

  public static final Set<Subsystem> REAL_SUBSYSTEMS =
      EnumSet.of(Subsystem.INTAKE, Subsystem.SHOOTER);
}
