package frc.robot.shooter;

/** Simulated serializer motor. */
public class SerializerMotorIOSim implements SerializerMotorIO {

  private double velocityRotationsPerSecond;

  @Override
  public void configure() {}

  @Override
  public void update(SerializerMotorIOValues values) {
    values.velocityRotationsPerSecond = this.velocityRotationsPerSecond;
    values.currentAmps = 0.0;
  }

  @Override
  public void setSetpoint(double velocityRotationsPerSecond) {
    this.velocityRotationsPerSecond = velocityRotationsPerSecond;
  }
}
