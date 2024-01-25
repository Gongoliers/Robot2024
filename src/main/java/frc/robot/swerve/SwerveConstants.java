package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.MotorCurrentLimits;
import frc.lib.PIDFConstants;

/** Constants for the swerve subsystem. */
public class SwerveConstants {

  /** Constants for the MK4i COTS module. */
  public static class MK4iConstants {
    /** Gearing between the steer motor and the wheel. */
    public static final double STEER_GEARING = 150.0 / 7.0;

    /** Moment of inertia of the wheel when steering in joules kilograms meters squared. */
    public static final double STEER_MOI = 0.004; // TODO

    /** Gearing between the drive motor and the wheel. */
    public static final double DRIVE_GEARING = 6.75;

    /** Diameter of the MK4i's wheels in meters. */
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

    /** Actual distance driven during the odometry test in meters. */
    public static final double ODOMETRY_TEST_ACTUAL_DISTANCE = 6.985;

    /** Reported distance driven during the odometry test in meters. */
    public static final double ODOMETRY_TEST_REPORTED_DISTANCE = 8.219;

    /** Conversion between odometry distance and actual distance travelled. */
    public static final double ODOMETRY_ERROR =
        ODOMETRY_TEST_ACTUAL_DISTANCE / ODOMETRY_TEST_REPORTED_DISTANCE;

    /** Conversion between wheel rotations and distances in meters. */
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI * ODOMETRY_ERROR;
  }

  /** Module X offset in meters. */
  public static final double X_OFFSET = Units.inchesToMeters(11.375);

  /** Module Y offset in meters. */
  public static final double Y_OFFSET = Units.inchesToMeters(11.375);

  /** Swerve's CAN bus. */
  public static final String SWERVE_BUS = "swerve";

  /** Module configuration for the north west swerve module. */
  public static final SwerveModuleConfig NORTH_WEST_MODULE_CONFIG =
      new SwerveModuleConfig(
          new SwerveModuleCAN(2, 1, 3, SWERVE_BUS),
          new Translation2d(X_OFFSET, Y_OFFSET),
          Rotation2d.fromRotations(-0.179688));

  /** Module configuration for the north east swerve module. */
  public static final SwerveModuleConfig NORTH_EAST_MODULE_CONFIG =
      new SwerveModuleConfig(
          new SwerveModuleCAN(5, 4, 6, SWERVE_BUS),
          new Translation2d(X_OFFSET, -Y_OFFSET),
          Rotation2d.fromRotations(-0.951904));

  /** Module configuration for the south east swerve module. */
  public static final SwerveModuleConfig SOUTH_EAST_MODULE_CONFIG =
      new SwerveModuleConfig(
          new SwerveModuleCAN(9, 8, 10, SWERVE_BUS),
          new Translation2d(-X_OFFSET, -Y_OFFSET),
          Rotation2d.fromRotations(-0.774568));

  /** Module configuration for the south west swerve module. */
  public static final SwerveModuleConfig SOUTH_WEST_MODULE_CONFIG =
      new SwerveModuleConfig(
          new SwerveModuleCAN(12, 11, 13, SWERVE_BUS),
          new Translation2d(-X_OFFSET, Y_OFFSET),
          Rotation2d.fromRotations(-0.954346));

  /**
   * Calculates the maximum attainable open loop speed in meters per second.
   *
   * @param rotorVelocityRotationsPerSecondAt12Volts the rotor velocity of the drive motor in
   *     rotations per second when the motor is supplied 12 volts.
   * @return the maximum attainable open loop speed in meters per second.
   */
  private static double calculateMaximumAttainableSpeed(
      double rotorVelocityRotationsPerSecondAt12Volts) {
    return rotorVelocityRotationsPerSecondAt12Volts
        / MK4iConstants.DRIVE_GEARING
        * MK4iConstants.WHEEL_CIRCUMFERENCE;
  }

  /** Maximum attainable speed in meters per second. */
  public static final double MAXIMUM_ATTAINABLE_SPEED = calculateMaximumAttainableSpeed(108.2);

  /** Maximum speed in meters per second. */
  public static final double MAXIMUM_SPEED = MAXIMUM_ATTAINABLE_SPEED;

  /**
   * Calculates an acceleration using a ramp duration.
   *
   * @param maximumSpeed the maximum speed in units per second.
   * @param rampDurationSeconds the desired duration to ramp from no speed to full speed.
   * @return the acceleration in units per second per second.
   */
  public static double calculateAcceleration(double maximumSpeed, double rampDurationSeconds) {
    return maximumSpeed / rampDurationSeconds;
  }

  /** Maximum acceleration in meters per second per second. */
  public static final double MAXIMUM_ACCELERATION = calculateAcceleration(MAXIMUM_SPEED, 0.1);

  /** Maximum attainable rotational speed in rotations per second. */
  public static final double MAXIMUM_ATTAINABLE_ROTATION_SPEED =
      NORTH_WEST_MODULE_CONFIG.position().getNorm() * MAXIMUM_ATTAINABLE_SPEED;

  /** Maximum rotational speed while snapping to heading in rotations per second. */
  public static final double MAXIMUM_ROTATION_SPEED = 0.5;

  /** Maximum acceleration in rotations per second per second. */
  public static final double MAXIMUM_ROTATION_ACCELERATION =
      calculateAcceleration(MAXIMUM_ROTATION_SPEED, 0.1);

  public static class DriveMotorConstants {
    /** If true, use open-loop control. */
    public static final boolean OPEN_LOOP = true;

    /** Feedback proportional gain in volts per meter per second. */
    public static final double FEEDBACK_KP = 1.0;

    /** Feedforward static constant in volts. */
    public static final double FEEDFORWARD_KS = 0.139;

    /** Feedforward velocity constant in volts per meter per second. */
    // TODO Tune, make sure to convert volts per rotation per second to volts per meter per second
    public static final double FEEDFORWARD_KV = 0.652628161;

    /** Current limits for the drive motor. */
    // TODO Copied from Citrus Circuits 2023 release
    public static final MotorCurrentLimits CURRENT_LIMITS =
        new MotorCurrentLimits(300.0, 40.0, 100.0, 1.0);
  }

  /** Constants for steer motor PIDF position controllers. */
  public static final PIDFConstants STEER_PIDF_CONSTANTS = new PIDFConstants();

  static {
    STEER_PIDF_CONSTANTS.kP = 48.0; // volts per rotation
    STEER_PIDF_CONSTANTS.kD = 0.25; // volts per rotation per second
    STEER_PIDF_CONSTANTS.kPositionTolerance = Units.degreesToRotations(3);
    STEER_PIDF_CONSTANTS.kVelocityConstraint = 10.0; // rotations per second
    STEER_PIDF_CONSTANTS.kAccelerationConstraint = 64.0; // rotations per second per second
    STEER_PIDF_CONSTANTS.kS = 0.16; // volts
    STEER_PIDF_CONSTANTS.kV = 0.407363; // volts per rotation per second
  }
}
