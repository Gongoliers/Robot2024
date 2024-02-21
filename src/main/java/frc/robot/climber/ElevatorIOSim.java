package frc.robot.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotConstants;
import frc.robot.climber.ClimberConstants.ElevatorConstants;

/** Simulated elevator. */
public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor motor;

  private final ElevatorSim elevatorSim;

  private final PIDController feedback;

  private final ElevatorFeedforward feedforward;

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
            true,
            ElevatorConstants.MIN_HEIGHT);

    // TODO
    feedback = new PIDController(1.0, 0.0, 0.0);

    // TODO
    feedforward = new ElevatorFeedforward(0, 0, 0);
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
    double feedbackVolts = feedback.calculate(elevatorSim.getPositionMeters(), positionMeters);

    double feedforwardVolts = feedforward.calculate(elevatorSim.getVelocityMetersPerSecond());

    elevatorSim.setInputVoltage(feedbackVolts + feedforwardVolts);
  }
}
