package frc.lib.config;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** Motion profile config. */
public class MotionProfileConfig {

  /** Maximum velocity. */
  private double maximumVelocity = 0.0;

  /** Maximum acceleration. */
  private double maximumAcceleration = 0.0;

  /**
   * Calculates an acceleration using a ramp duration.
   *
   * @param maximumSpeed the maximum speed in units per second.
   * @param desiredRampDurationSeconds the desired duration to ramp from no speed to full speed.
   * @return the acceleration in units per second per second.
   */
  public static double calculateAcceleration(
      double maximumSpeed, double desiredRampDurationSeconds) {
    return maximumSpeed / desiredRampDurationSeconds;
  }

  /**
   * Modifies this motion profile config's maximum velocity.
   *
   * @param maximumVelocity the maximum velocity.
   * @return this motion profile config.
   */
  public MotionProfileConfig withMaximumVelocity(double maximumVelocity) {
    this.maximumVelocity = maximumVelocity;
    return this;
  }

  /**
   * Modifies this motion profile config's maximum acceleration.
   *
   * @param maximumAcceleration the maximum acceleration.
   * @return this motion profile config.
   */
  public MotionProfileConfig withMaximumAcceleration(double maximumAcceleration) {
    this.maximumAcceleration = maximumAcceleration;
    return this;
  }

  /**
   * Returns the motion profile maximum velocity.
   *
   * @return the motion profile maximum velocity.
   */
  public double maximumVelocity() {
    return maximumVelocity;
  }

  /**
   * Returns the motion profile maximum acceleration.
   *
   * @return the motion profile maximum acceleration.
   */
  public double maximumAcceleration() {
    return maximumAcceleration;
  }

  /**
   * Creates a new velocity clamper using this motion profile config.
   * 
   * @return a new velocity clamper using this motion profile config.
   */
  public Function<Double, Double> createVelocityClamper() {
    return velocity -> MathUtil.clamp(velocity, -maximumVelocity, maximumVelocity);
  }

  /**
   * Creates a new acceleration limiter using this motion profile config.
   *
   * @return a new acceleration limiter using this motion profile config.
   */
  public SlewRateLimiter createAccelerationLimiter() {
    return new SlewRateLimiter(maximumAcceleration);
  }

  /**
   * Creates a new trapezoid profile using this motion profile config.
   *
   * @return a new trapezoid profile using this motion profile config.
   */
  public TrapezoidProfile createTrapezoidProfile() {
    return new TrapezoidProfile(new Constraints(maximumVelocity, maximumAcceleration));
  }
}
