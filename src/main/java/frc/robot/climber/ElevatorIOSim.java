package frc.robot.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotConstants;
import frc.robot.climber.ClimberConstants.ElevatorConstants;

/** Simulated elevator. */
public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor motor;

  private final ElevatorSim elevatorSim;

  public ElevatorIOSim() {
    motor = DCMotor.getNEO(1);

    elevatorSim =
        new ElevatorSim(
            motor,
            ElevatorConstants.GEARING,
            ElevatorConstants.MASS,
            ElevatorConstants.DRUM_DIAMETER,
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT,
            false,
            ElevatorConstants.MIN_HEIGHT);
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
  public void setVoltage(double volts) {
    elevatorSim.setInputVoltage(volts);
  }
}
