package frc.robot.climber;

import frc.lib.CAN;

/** Helper class for creating hardware for the climber subsystem. */
public class ClimberFactory {

  /**
   * Creates an elevator.
   *
   * @return an elevator.
   */
  public static ElevatorIO createElevator(CAN can, boolean inverted) {
    return new ElevatorIOSim();
  }
}
