package frc.robot.intake;

/** Simulated roller motor. */
public class RollerMotorIOSim implements RollerMotorIO {

  private double velocityRotationsPerSecond = 0.0;

  @Override
  public void configure() {}

  @Override
  public void update(RollerMotorIOValues values) {
    values.velocityRotationsPerSecond = this.velocityRotationsPerSecond;
    values.currentAmps = 0.0;
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    this.velocityRotationsPerSecond = velocityRotationsPerSecond;
  }
}
