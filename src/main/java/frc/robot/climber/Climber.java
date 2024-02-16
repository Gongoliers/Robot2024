package frc.robot.climber;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.climber.ClimberConstants.ElevatorConstants;
import frc.robot.climber.ElevatorIO.ElevatorIOValues;
import frc.robot.climber.LinearActuatorIO.LinearActuatorIOValues;

/** Subsystem class for the climber subsystem. */
public class Climber extends Subsystem {

  /** Instance variable for the climber subsystem singleton. */
  private static Climber instance = null;

  /** Elevator. */
  private final ElevatorIO elevator;

  /** Elevator values. */
  private final ElevatorIOValues elevatorValues = new ElevatorIOValues();

  /** Linear actuator. */
  private final LinearActuatorIO linearActuator;

  /** Linear actuator values. */
  private final LinearActuatorIOValues linearActuatorValues = new LinearActuatorIOValues();

  /** Creates a new instance of the climber subsystem. */
  private Climber() {
    elevator = ClimberFactory.createElevator();
    linearActuator = ClimberFactory.createLinearActuator();

    elevator.setPosition(ElevatorConstants.MIN_HEIGHT);
    linearActuator.setPosition(0);
  }

  /**
   * Gets the instance of the climber subsystem.
   *
   * @return the instance of the climber subsystem.
   */
  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout elevator = Telemetry.addColumn(tab, "Elevator");

    elevator.addDouble("Position (m)", () -> elevatorValues.positionMeters);

    ShuffleboardLayout linearActuator = Telemetry.addColumn(tab, "Linear Actuator");

    linearActuator.addDouble("Position (%)", () -> linearActuatorValues.positionPercent);
  }
}
