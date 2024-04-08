package frc.robot.intake;

import frc.lib.CAN;
import frc.lib.PIDFConstants;

/** Constants for the intake subsystem. */
public class IntakeConstants {
  /** Constants for the front roller. */
  public static class FrontRollerConstants {
    /** Front roller's CAN. */
    public static final CAN CAN = new CAN(50);

    /** Front roller's PIDF constants. */
    public static final PIDFConstants PIDF = new PIDFConstants();
    static {
      PIDF.kS = 0.13;
      PIDF.kV = 0.1683;
    }
  }

  /** Constants for the back roller. */
  public static class BackRollerConstants {
    /** Back roller's CAN. */
    public static final CAN CAN = new CAN(50);

    /** Back roller's PIDF constants. */
    public static final PIDFConstants PIDF = new PIDFConstants();
    static {
      PIDF.kS = 0.13;
      PIDF.kV = 0.1759;
    }
  }

  /** Constants for the roller motor. */
  public static class RollerConstants {
    /** Velocity to apply when intaking in rotations per second. */
    public static final double INTAKE_VELOCITY = 34;

    /** Maximum speed of the roller in rotations per second. */
    public static final double MAXIMUM_SPEED = 67;

    /** Speed tolerance of the roller in rotations per second. */
    public static final double SPEED_TOLERANCE = 2.5;
  }
}
