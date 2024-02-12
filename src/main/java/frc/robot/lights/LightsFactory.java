package frc.robot.lights;

import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Helper class for creating hardware for the lights subsystem. */
public class LightsFactory {

  /**
   * Creates an LED controller.
   *
   * @return an LED controller.
   */
  public static LEDControllerIO createLEDController() {
    if (Robot.isReal() && RobotConstants.REAL_SUBSYSTEMS.contains(Subsystem.LIGHTS))
      return new LEDControllerIOCANdle();

    return new LEDControllerIOSim();
  }
}
