package frc.robot.lights;

import frc.robot.Robot;

/** Helper class for creating hardware for the lights subsystem. */
public class LightsFactory {

  /**
   * Creates an LED controller.
   *
   * @return an LED controller.
   */
  public static LEDControllerIO createLEDController() {
    //if (Robot.isReal()) return new LEDControllerIOCANdle();

    return new LEDControllerIOSim();
  }
}
