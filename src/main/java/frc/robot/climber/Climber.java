package frc.robot.climber;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.climber.ClimberConstants.ElevatorConstants;
import frc.robot.climber.ElevatorIO.ElevatorIOValues;

/** Subsystem class for the climber subsystem. */
public class Climber extends Subsystem {

  /** Instance variable for the climber subsystem singleton. */
  private static Climber instance = null;

  /** Elevator. */
  private final ElevatorIO elevator;

  /** Elevator values. */
  private final ElevatorIOValues elevatorValues = new ElevatorIOValues();

  /** Creates a new instance of the climber subsystem. */
  private Climber() {
    elevator = ClimberFactory.createElevator();

    elevator.setPosition(ElevatorConstants.MIN_HEIGHT);
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
  }
}
