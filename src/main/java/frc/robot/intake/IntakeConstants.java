package frc.robot.intake;

import frc.lib.CAN;
import frc.lib.PIDFConstants;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOConstants;

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

    /** Front roller's controller constants. */
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS =
        new VelocityControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = true;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 24.0 / 16.0;
    }

    public static final double INTAKE_SPEED = 34;

    /** Maximum speed of the roller in rotations per second. */
    public static final double MAXIMUM_SPEED = 67;
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

    /** Back roller's controller constants. */
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS =
        new VelocityControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = true;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 24.0 / 16.0;
    }

    public static final double INTAKE_SPEED = 34;

    /** Maximum speed of the roller in rotations per second. */
    public static final double MAXIMUM_SPEED = 67;
  }
}
