package frc.robot;

import java.util.EnumSet;
import java.util.Set;

/** Constants for the entire robot. */
public class RobotConstants {

  /** Number of robot periodic calls per second. */
  public static final double PERIODIC_RATE = 50;

  /** Duration of each robot periodic call in seconds. */
  public static final double PERIODIC_DURATION = 1.0 / PERIODIC_RATE;

  /** Subsystems. */
  public enum Subsystem {
    ARM,
    INTAKE,
    ODOMETRY,
    SHOOTER,
    SWERVE,
  }

  /** Real subsystems. */
  public static final Set<Subsystem> REAL_SUBSYSTEMS =
      EnumSet.of(
          Subsystem.ARM, Subsystem.INTAKE, Subsystem.ODOMETRY, Subsystem.SHOOTER, Subsystem.SWERVE);
}
