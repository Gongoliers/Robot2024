package frc.robot.lights;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.CAN;

/** Constants for the lights subsystem. */
public class LightsConstants {

  /** LED controller's CAN identifier. */
  public static final CAN CANDLE_ID = new CAN(20);

  /** Constants for animations. */
  public static class Animations {

    /** Turn off LEDs. */
    public static final SolidLEDAnimation OFF = new SolidLEDAnimation(Color.kBlack);

    /** Follow the robot signal light. */
    public static final BooleanLEDAnimation FOLLOW_RSL =
        new BooleanLEDAnimation(RobotController::getRSLState, Color.kOrangeRed);
  }
}
