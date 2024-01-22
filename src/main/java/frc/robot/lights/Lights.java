package frc.robot.lights;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.Subsystem;
import frc.robot.lights.LEDControllerIO.LEDControllerIOValues;
import frc.robot.lights.LightsConstants.Animations;
import java.util.Map;

/** Subsystem class for the lights subsystem. */
public class Lights extends Subsystem {

  /** Instance variable for the lights subsystem singleton */
  private static Lights instance = null;

  /** LED controller. */
  private final LEDControllerIO ledController;

  /** LED controller values. */
  private final LEDControllerIOValues ledControllerValues = new LEDControllerIOValues();

  /** Widget for displaying color on Shuffleboard. */
  private SimpleWidget lightWidget;

  private LEDAnimation animation;

  /** Creates a new instance of the lights subsystem. */
  private Lights() {
    ledController = LightsFactory.createLEDController();

    animation = Animations.FOLLOW_RSL;
  }

  /**
   * Gets the instance of the lights subsystem.
   *
   * @return the instance of the lights subsystem.
   */
  public static Lights getInstance() {
    if (instance == null) {
      instance = new Lights();
    }

    return instance;
  }

  @Override
  public void periodic() {
    ledController.update(ledControllerValues);

    setColor(animation.getColor());

    if (lightWidget != null) {
      lightWidget = lightWidget.withProperties(Map.of("colorWhenTrue", getColor().toHexString()));
    }
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    lightWidget =
        tab.add("Light Color", true)
            .withProperties(
                Map.of("colorWhenFalse", "#000000", "colorWhenTrue", getColor().toHexString()));
  }

  /**
   * Gets the color of the lights.
   *
   * @return the color of the lights.
   */
  public Color getColor() {
    return new Color(ledControllerValues.red, ledControllerValues.blue, ledControllerValues.green);
  }

  /**
   * Sets the color of the lights.
   *
   * @param color the color of the lights.
   */
  public void setColor(Color color) {
    ledController.setColor(color.red, color.green, color.blue);
  }

  /**
   * Sets the animation of the lights.
   *
   * @param animation the animation of the lights.
   */
  public void setAnimation(LEDAnimation animation) {
    this.animation = animation;
  }
}
