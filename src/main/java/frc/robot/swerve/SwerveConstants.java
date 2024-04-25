package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.MotionProfileCalculator;
import frc.lib.config.FeedbackControllerConfig;
import frc.lib.config.FeedforwardControllerConfig;
import frc.lib.config.MechanismConfig;
import frc.lib.config.MotorConfig;

/** Constants for the swerve subsystem. */
public class SwerveConstants {

  /** Constants for the MK4i COTS module. */
  public static class MK4iConstants {
    /** Gearing between the steer motor and the wheel. */
    public static final double STEER_GEARING = 150.0 / 7.0;

    /** Gearing between the drive motor and the wheel. */
    public static final double DRIVE_GEARING = 6.75;

    /** Diameter of the MK4i's wheels in meters. */
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

    /** Actual distance driven during the odometry test in meters. */
    public static final double ODOMETRY_TEST_ACTUAL_DISTANCE = 5.49;

    /** Reported distance driven during the odometry test in meters. */
    public static final double ODOMETRY_TEST_REPORTED_DISTANCE = 5.0;

    /** Conversion between odometry distance and actual distance travelled. */
    public static final double ODOMETRY_ERROR =
        ODOMETRY_TEST_REPORTED_DISTANCE / ODOMETRY_TEST_ACTUAL_DISTANCE;

    /** Conversion between wheel rotations and distances in meters. */
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI * ODOMETRY_ERROR;
  }

  /** Module X offset in meters. */
  public static final double X_OFFSET = Units.inchesToMeters(10.375);

  /** Module Y offset in meters. */
  public static final double Y_OFFSET = Units.inchesToMeters(10.375);

  /** Swerve's CAN bus. */
  public static final String SWERVE_BUS = "swerve";

  /** Module configuration for the north west swerve module. */
  public static final SwerveModuleConfig NORTH_WEST_MODULE_CONFIG =
      new SwerveModuleConfig(
          new SwerveModuleCAN(16, 8, 24, SWERVE_BUS),
          new Translation2d(X_OFFSET, Y_OFFSET),
          Rotation2d.fromRotations(-0.084717).unaryMinus());

  /** Module configuration for the north east swerve module. */
  public static final SwerveModuleConfig NORTH_EAST_MODULE_CONFIG =
      new SwerveModuleConfig(
          new SwerveModuleCAN(18, 16, 30, SWERVE_BUS),
          new Translation2d(X_OFFSET, -Y_OFFSET),
          Rotation2d.fromRotations(0.196777).unaryMinus());

  /** Module configuration for the south east swerve module. */
  public static final SwerveModuleConfig SOUTH_EAST_MODULE_CONFIG =
      new SwerveModuleConfig(
          new SwerveModuleCAN(22, 12, 26, SWERVE_BUS),
          new Translation2d(-X_OFFSET, -Y_OFFSET),
          Rotation2d.fromRotations(0.276611).unaryMinus());

  /** Module configuration for the south west swerve module. */
  public static final SwerveModuleConfig SOUTH_WEST_MODULE_CONFIG =
      new SwerveModuleConfig(
          new SwerveModuleCAN(20, 10, 28, SWERVE_BUS),
          new Translation2d(-X_OFFSET, Y_OFFSET),
          Rotation2d.fromRotations(-0.276855).plus(Rotation2d.fromDegrees(180)).unaryMinus());

  /**
   * Calculates the maximum attainable open loop speed in meters per second.
   *
   * @param driveRotorVelocityRotationsPerSecondAt12Volts the rotor velocity of the drive motor in
   *     rotations per second when the motor is supplied 12 volts.
   * @return the maximum attainable open loop speed in meters per second.
   */
  private static double calculateMaximumAttainableSpeed(
      double driveRotorVelocityRotationsPerSecondAt12Volts) {
    return driveRotorVelocityRotationsPerSecondAt12Volts
        / MK4iConstants.DRIVE_GEARING
        * MK4iConstants.WHEEL_CIRCUMFERENCE;
  }

  /** Maximum attainable speed in meters per second. */
  public static final double MAXIMUM_ATTAINABLE_SPEED = calculateMaximumAttainableSpeed(100);

  /** Maximum speed in meters per second. */
  public static final double MAXIMUM_SPEED = MAXIMUM_ATTAINABLE_SPEED;

  /** Maximum acceleration in meters per second per second. */
  public static final double MAXIMUM_ACCELERATION =
      MotionProfileCalculator.calculateAcceleration(MAXIMUM_SPEED, 0.25);

  /** Maximum attainable rotational speed. */
  public static final Rotation2d MAXIMUM_ATTAINABLE_ROTATION_SPEED =
      Rotation2d.fromRotations(
          MAXIMUM_ATTAINABLE_SPEED / NORTH_WEST_MODULE_CONFIG.position().getNorm());

  /** Maximum rotational speed. */
  public static final Rotation2d MAXIMUM_ROTATION_SPEED = Rotation2d.fromRotations(0.5);

  /** Maximum acceleration in rotations per second per second. */
  public static final Rotation2d MAXIMUM_ROTATION_ACCELERATION =
      Rotation2d.fromRotations(
          MotionProfileCalculator.calculateAcceleration(
              MAXIMUM_ROTATION_SPEED.getRotations(), 0.1));

  /** Maximum rotation speed and rotation acceleration. */
  public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAXIMUM_ROTATION_SPEED.getRotations(), MAXIMUM_ROTATION_ACCELERATION.getRotations());

  /** Rotation motion profile. */
  public static final TrapezoidProfile ROTATION_MOTION_PROFILE =
      new TrapezoidProfile(ROTATION_CONSTRAINTS);

  /**
   * Converts a kV constant in volts per rotor rotation per second to a kV constant in volts per
   * meters per second.
   *
   * @param voltsPerRotorRotationPerSecond a kV constant in volts per rotor rotations per second.
   * @return a kV constant in volts per rotor rotations per second.
   */
  private static double calculateKv(double voltsPerRotorRotationPerSecond) {
    return voltsPerRotorRotationPerSecond
        * MK4iConstants.DRIVE_GEARING
        / MK4iConstants.WHEEL_CIRCUMFERENCE;
  }

  /** Drive motor config. */
  public static final MechanismConfig DRIVE_CONFIG =
      new MechanismConfig()
          .withMotorConfig(
              new MotorConfig()
                  .withCCWPositive(false)
                  .withCurrentLimit(40)
                  .withMotorToMechanismRatio(MK4iConstants.DRIVE_GEARING))
          .withFeedforwardConfig(
              new FeedforwardControllerConfig()
                  .withStaticFeedforward(0.14) // volts
                  .withVelocityFeedforward(calculateKv(0.12)) // volts per meter per second
              );

  /** Steer motor config. */
  public static final MechanismConfig STEER_CONFIG =
      new MechanismConfig()
          .withMotorConfig(
              new MotorConfig()
                  .withCCWPositive(false)
                  .withCurrentLimit(20)
                  .withMotorToMechanismRatio(MK4iConstants.STEER_GEARING))
          .withFeedforwardConfig(
              new FeedforwardControllerConfig().withStaticFeedforward(0.205) // volts
              )
          .withFeedbackConfig(
              new FeedbackControllerConfig()
                  .withContinuousInput(true)
                  .withProportionalGain(54.0) // volts per rotation
                  .withDerivativeGain(0.16) // volts per rotation per second
                  .withPositionTolerance(Units.degreesToRotations(1)) // rotations
              );
}
