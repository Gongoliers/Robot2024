package frc.robot.intake;

import frc.lib.CAN;

/** Constants for the intake subsystem. */
public class IntakeConstants {

  /** Constants for the intake motor. */
  public static class IntakeMotorConstants {
    /** Intake motor's CAN identifier. */
    public static final CAN ID = new CAN(62);

    /** If true, invert the intake motor. */
    public static final boolean IS_INVERTED = false;

    /** Voltage to apply when intaking in volts. */
    public static final double INTAKE_VOLTAGE = -7.2;
  }
}
