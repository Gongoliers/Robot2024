package frc.robot.climber;

/** Helper class for creating hardware for the climber subsystem. */
public class ClimberFactory {

  /**
   * Creates an elevator.
   *
   * @return an elevator.
   */
  public static ElevatorIO createElevator() {
    // TODO
    // if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.CLIMBER)) {
    //     return new ElevatorIONeo();
    // }

    return new ElevatorIOSim();
  }
}
