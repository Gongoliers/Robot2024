package frc.robot.climber;

/** Simulated linear actuator. */
public class LinearActuatorIOSim implements LinearActuatorIO {

  private double positionPercent;

  @Override
  public void configure() {}

  @Override
  public void update(LinearActuatorIOValues values) {
    values.positionPercent = this.positionPercent;
  }

  @Override
  public void setPosition(double positionPercent) {
    positionPercent = 0.0;
  }

  @Override
  public void setSetpoint(double positionPercent) {
    positionPercent = 0.0;
  }
}
