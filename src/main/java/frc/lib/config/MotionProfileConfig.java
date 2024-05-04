package frc.lib.config;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.Objects;
import java.util.function.Function;

/** Motion profile config. */
public record MotionProfileConfig(double maximumVelocity, double maximumAcceleration) {

  public MotionProfileConfig {
    Objects.requireNonNull(maximumVelocity);
    Objects.requireNonNull(maximumAcceleration);
  }

  public static final class MotionProfileConfigBuilder {

    private double maximumVelocity;

    private double maximumAcceleration;

    public static MotionProfileConfigBuilder defaults() {
      return new MotionProfileConfigBuilder(0.0, 0.0);
    }

    public static MotionProfileConfigBuilder from(MotionProfileConfig motionProfileConfig) {
      return new MotionProfileConfigBuilder(
          motionProfileConfig.maximumVelocity, motionProfileConfig.maximumAcceleration);
    }

    private MotionProfileConfigBuilder(double maximumVelocity, double maximumAcceleration) {
      this.maximumVelocity = maximumVelocity;
      this.maximumAcceleration = maximumAcceleration;
    }

    public MotionProfileConfigBuilder maximumVelocity(double maximumVelocity) {
      this.maximumVelocity = maximumVelocity;
      return this;
    }

    public MotionProfileConfigBuilder maximumAcceleration(double maximumAcceleration) {
      this.maximumAcceleration = maximumAcceleration;
      return this;
    }

    public MotionProfileConfig build() {
      return new MotionProfileConfig(maximumVelocity, maximumAcceleration);
    }
  }

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
