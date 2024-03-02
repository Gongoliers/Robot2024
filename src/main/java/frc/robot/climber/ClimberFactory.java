package frc.robot.climber;

import frc.lib.CAN;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the climber subsystem. */
public class ClimberFactory {

  /**
   * Creates an elevator.
   *
   * @return an elevator.
   */
  public static ElevatorIO createElevator(CAN can, boolean inverted) {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.CLIMBER)) {
      return new ElevatorIOSparkMax(can, inverted);
    }

    return new ElevatorIOSim();
  }
}
