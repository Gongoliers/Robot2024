package frc.lib.sensor;

import frc.robot.RobotConstants;
import java.util.function.DoubleSupplier;

/** Simulated gyroscope. */
public class GyroscopeIOSim implements GyroscopeIO {

  /** Yaw velocity supplier in rotations per second */
  private final DoubleSupplier yawVelocityRotationsPerSecond;

  /** Yaw in rotations. */
  private double yawRotations = 0.0;

  /** Creates a new simulated gyroscope. */
  public GyroscopeIOSim(DoubleSupplier yawVelocityRotationsPerSecond) {
    this.yawVelocityRotationsPerSecond = yawVelocityRotationsPerSecond;
  }

  @Override
  public void configure() {}

  @Override
  public void update(GyroscopeIOValues values) {
    yawRotations += yawVelocityRotationsPerSecond.getAsDouble() * RobotConstants.PERIODIC_DURATION;

    values.yawRotations = yawRotations;

    values.yawVelocityRotations = yawVelocityRotationsPerSecond.getAsDouble();
  }

  @Override
  public void setYaw(double yawRotations) {
    this.yawRotations = yawRotations;
  }
}
