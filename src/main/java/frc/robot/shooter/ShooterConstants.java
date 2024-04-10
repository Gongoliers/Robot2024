package frc.robot.shooter;

import frc.lib.CAN;
import frc.lib.PIDFConstants;
import frc.lib.controller.VelocityControllerIO.VelocityControllerIOConstants;

/** Constants for the shooter subsystem. */
public class ShooterConstants {

  /** Constants for the serializer motor used in the shooter subsystem. */
  public static class SerializerConstants {
    /** Serializer's CAN. */
    public static final CAN CAN = new CAN(42);

    /** Serializer's PIDF constants. */
    public static final PIDFConstants PIDF = new PIDFConstants();

    static {
      PIDF.kS = 0.14;
      PIDF.kV = 0.2617;
    }

    /** Serializer's controller constants. */
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS =
        new VelocityControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = false;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 36.0 / 16.0;
    }

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 45.319;
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** Flywheel's CAN. */
    public static final CAN CAN = new CAN(44);

    /** Flywheel's PIDF constants. */
    public static final PIDFConstants PIDF = new PIDFConstants();

    static {
      PIDF.kS = 0.14;
      PIDF.kV = 0.2539;
    }

    /** Flywheel's controller constants. */
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS =
        new VelocityControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = false;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 28.0 / 16.0;
    }

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 46.711;
  }
}
