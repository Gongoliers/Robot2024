package frc.robot.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.lib.CAN;
import frc.lib.MotionProfileCalculator;
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
      CONTROLLER_CONSTANTS.ccwPositive = true;
      CONTROLLER_CONSTANTS.neutralBrake = false;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 36.0 / 16.0;
    }

    public static final double INTAKE_SPEED = 34;

    public static final double PULL_SPEED = -10;

    public static final double EJECT_SPEED = -44;

    public static final double SLOW_FEED_SPEED = 20;

    public static final double FAST_FEED_SPEED = 44;

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 45.319;

    public static final SlewRateLimiter ACCELERATION_LIMITER =
        new SlewRateLimiter(MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.1));

    public static final double NOTE_AMPS = 20;
  }

  /** Constants for the flywheel motor used in the shooter subsystem. */
  public static class FlywheelConstants {
    /** Flywheel's CAN. */
    public static final CAN CAN = new CAN(44);

    /** Flywheel's PIDF constants. */
    public static final PIDFConstants PIDF = new PIDFConstants();

    static {
      PIDF.kS = 0.14;
      PIDF.kV = 0.2;
    }

    /** Flywheel's controller constants. */
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS =
        new VelocityControllerIOConstants();

    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = true;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 28.0 / 16.0;
    }

    public static final double PULL_SPEED = -20;

    public static final double SPEAKER_SPEED = 60;

    public static final double PODIUM_SPEED = 60;

    public static final double LOB_SPEED = 60;

    public static final double SKIM_SPEED = 60;

    public static final double AMP_SPEED = 20;

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 60;

    public static final SlewRateLimiter ACCELERATION_LIMITER =
        new SlewRateLimiter(MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.1));

    public static final int NOTE_AMPS = 40;
  }
}
