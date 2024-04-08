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
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS = new VelocityControllerIOConstants();
    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = true;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 36.0 / 16.0;
    }

    /** Velocity to apply while intaking in rotations per second. */
    public static final double INTAKE_VELOCITY = 34;

    /** Velocity to apply while pulling in rotations per second. */
    public static final double PULL_VELOCITY = -20;

    /** Velocity to apply while serializing in rotations per second. */
    public static final double SERIALIZE_VELOCITY = 20;

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 45.319;

    /** Speed tolerance in rotations per second. */
    public static final double SPEED_TOLERANCE = 2.5;
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
    public static final VelocityControllerIOConstants CONTROLLER_CONSTANTS = new VelocityControllerIOConstants();
    static {
      CONTROLLER_CONSTANTS.ccwPositive = false;
      CONTROLLER_CONSTANTS.neutralBrake = true;
      CONTROLLER_CONSTANTS.sensorToMechanismRatio = 36.0 / 16.0;
    }

    /** Velocity to apply while shooting into the speaker in rotations per second. */
    public static final double SPEAKER_VELOCITY = 44;

    /** Velocity to apply while passing in rotations per second. */
    public static final double PASS_VELOCITY = 12;

    /** Velocity to apply while shooting into the amp in rotations per second. */
    public static final double AMP_VELOCITY = 20;

    /** Maximum speed in rotations per second. */
    public static final double MAXIMUM_SPEED = 46.711;

    /** Maximum acceleration in rotations per second per second. */
    public static final double MAXIMUM_ACCELERATION = MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.25);

    /** Acceleration limiter. */
    public static final SlewRateLimiter ACCELERATION_LIMITER = new SlewRateLimiter(MAXIMUM_ACCELERATION);

    /** Speed tolerance in rotations per second. */
    public static final double SPEED_TOLERANCE = 2.5;
  }
}
