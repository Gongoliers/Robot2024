package frc.robot.odometry;

import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;
import java.util.function.DoubleSupplier;

/** Simulated gyroscope. */
public class GyroscopeIOSim implements GyroscopeIO {

  /** Gyroscope's yaw. */
  private double yawRotations;

  private final DoubleSupplier yawVelocityRotations;

  /** Creates a new simulated gyroscope. */
  public GyroscopeIOSim(Odometry odometry) {
    yawRotations = 0.0;

    yawVelocityRotations =
        () -> Units.radiansToRotations(odometry.getVelocity().dtheta);
  }

  @Override
  public void configure() {}

  @Override
  public void update(GyroscopeIOValues values) {
    yawRotations +=
        yawVelocityRotations.getAsDouble() * RobotConstants.PERIODIC_DURATION;

    values.yawRotations = yawRotations;

    values.yawVelocityRotations = yawVelocityRotations.getAsDouble();
  }

  @Override
  public void setYaw(double yawRotations) {
    this.yawRotations = yawRotations;
  }
}
