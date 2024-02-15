package frc.robot.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotConstants;

/** Simulated elevator. */
public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor motor;

  private final ElevatorSim elevatorSim;

  public ElevatorIOSim() {
    motor = DCMotor.getVex775Pro(1);

    double gearing = 1.0;

    double massKg = 1.0;

    double drumDiameterMeters = 1.0;

    double minHeightMeters = 1.0;

    double maxHeightMeters = 1.0;

    elevatorSim =
        new ElevatorSim(
            motor,
            gearing,
            massKg,
            drumDiameterMeters,
            minHeightMeters,
            maxHeightMeters,
            true,
            minHeightMeters);
  }

  @Override
  public void configure() {}

  @Override
  public void update(ElevatorIOValues values) {
    elevatorSim.update(RobotConstants.PERIODIC_DURATION);

    values.positionMeters = elevatorSim.getPositionMeters();
  }

  @Override
  public void setPosition(double positionMeters) {
    elevatorSim.setState(positionMeters, 0);
  }

  @Override
  public void setSetpoint(double positionMeters) {
    // TODO
    setPosition(positionMeters);
  }
}
