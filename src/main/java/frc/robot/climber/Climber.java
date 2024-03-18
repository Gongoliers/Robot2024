package frc.robot.climber;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.climber.ClimberConstants.ElevatorConstants;
import frc.robot.climber.ElevatorIO.ElevatorIOValues;

/** Subsystem class for the climber subsystem. */
public class Climber extends Subsystem {

  /** Instance variable for the climber subsystem singleton. */
  private static Climber instance = null;

  /** Elevator. */
  private final ElevatorIO eastElevator;

  /** Elevator values. */
  private final ElevatorIOValues eastElevatorValues = new ElevatorIOValues();

  /** Elevator. */
  private final ElevatorIO westElevator;

  /** Elevator values. */
  private final ElevatorIOValues westElevatorValues = new ElevatorIOValues();

  /** Creates a new instance of the climber subsystem. */
  private Climber() {
    westElevator = new ElevatorIOSim();
    eastElevator = new ElevatorIOSim();

    westElevator.configure();
    eastElevator.configure();

    westElevator.setPosition(ElevatorConstants.MIN_HEIGHT);
    eastElevator.setPosition(ElevatorConstants.MIN_HEIGHT);
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
  public void periodic() {
    westElevator.update(westElevatorValues);
    eastElevator.update(eastElevatorValues);
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout west = Telemetry.addColumn(tab, "West");

    west.addDouble("Position (m)", () -> westElevatorValues.positionMeters);

    ShuffleboardLayout east = Telemetry.addColumn(tab, "East");

    east.addDouble("Position (m)", () -> eastElevatorValues.positionMeters);
  }

  public boolean westAtTop() {
    return westElevatorValues.positionMeters >= ElevatorConstants.MAX_HEIGHT;
  }

  public boolean westAtBottom() {
    return westElevatorValues.positionMeters <= ElevatorConstants.MIN_HEIGHT;
  }

  public boolean eastAtTop() {
    return eastElevatorValues.positionMeters >= ElevatorConstants.MAX_HEIGHT;
  }

  public boolean eastAtBottom() {
    return eastElevatorValues.positionMeters <= ElevatorConstants.MIN_HEIGHT;
  }

  public void stop() {
    westElevator.setVoltage(0);
    eastElevator.setVoltage(0);
  }

  public Command up() {
    return Commands.parallel(
            Commands.run(() -> westElevator.setVoltage(ElevatorConstants.UP_VOLTAGE))
                .until(this::westAtTop),
            Commands.run(() -> eastElevator.setVoltage(ElevatorConstants.UP_VOLTAGE))
                .until(this::eastAtTop))
        .finallyDo(this::stop);
  }

  public Command down() {
    return Commands.parallel(
            Commands.run(() -> westElevator.setVoltage(ElevatorConstants.DOWN_VOLTAGE))
                .until(this::westAtBottom),
            Commands.run(() -> eastElevator.setVoltage(ElevatorConstants.DOWN_VOLTAGE))
                .until(this::eastAtBottom))
        .finallyDo(this::stop);
  }
}
