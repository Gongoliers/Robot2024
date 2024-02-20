package frc.robot.intake;

import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.MotorCurrentLimits;

/** Constants for the intake subsystem. */
public class IntakeConstants {

  public static final double INTAKE_ROLLER_RADIUS = 0.5 * Units.inchesToMeters(1.375);

  /** Constants for the roller motor. */
  public static class RollerMotorConstants {
    /** Roller motor's CAN identifier. */
    public static final CAN ID = new CAN(4);

    /** If true, invert the roller motor. */
    public static final boolean IS_INVERTED = false;

    /** Gearing between the roller motor and the rollers. */
    public static final double GEARING = 4.5;

    /** Moment of inertia of the roller in joules kilograms meters squared. */
    public static final double MOI = 0.5343;

    /** Voltage to apply when intaking in volts. */
    // 12 volts produces enough speed to unscrew the dead axle
    public static final double INTAKE_VOLTAGE = 11;

    /** Voltage to apply when outtaking in volts. */
    public static final double OUTTAKE_VOLTAGE = -8;

    /** Current limits for the roller motor. */
    public static final MotorCurrentLimits CURRENT_LIMITS = new MotorCurrentLimits(40);

    /** Size of the current spike when intaking a note in amps. */
    public static final double NOTE_CURRENT = 18.0; // TODO

    /** Duration of the current spike when intaking a note. */
    public static final double STALL_DURATION = 0.35;
  }

  /** Constants for intake commands. */
  public static class IntakeCommandConstants {
    /** Delay between starting intaking and detecting notes in seconds. */
    public static final double NOTE_DETECTION_DELAY = 0.5;
  }
}
