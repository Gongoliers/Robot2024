package frc.robot.climber;

/** Simulated elevator. */
public class ElevatorIOSim implements ElevatorIO {

  private double positionMeters;

  @Override
  public void configure() {}

  @Override
  public void update(ElevatorIOValues values) {
    values.positionMeters = positionMeters;
  }

  @Override
  public void setPosition(double positionMeters) {
    this.positionMeters = positionMeters;
  }

  @Override
  public void setVoltage(double volts) {}
}
