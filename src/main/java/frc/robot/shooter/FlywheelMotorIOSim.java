package frc.robot.shooter;

/** Simulated flywheel motor. */
public class FlywheelMotorIOSim implements FlywheelMotorIO {

  private double velocityRotationsPerSecond;

  @Override
  public void configure() {}

  @Override
  public void update(FlywheelMotorIOValues values) {
    values.velocityRotationsPerSecond = this.velocityRotationsPerSecond;
    values.currentAmps = 0.0;
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    this.velocityRotationsPerSecond = velocityRotationsPerSecond;
  }
}
