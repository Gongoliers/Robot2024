package frc.robot.lights;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.Subsystem;
import frc.lib.Telemetry;
import frc.robot.lights.LEDControllerIO.LEDControllerIOValues;
import java.util.Map;

/** Subsystem class for the lights subsystem. */
public class Lights extends Subsystem {

  /** Instance variable for the lights subsystem singleton */
  private static Lights instance = null;

  /** LED controller. */
  private final LEDControllerIO ledController;

  /** LED controller values. */
  private final LEDControllerIOValues ledControllerValues = new LEDControllerIOValues();

  private SimpleWidget mainLightWidget;

  /** Creates a new instance of the lights subsystem. */
  private Lights() {
    ledController = LightsFactory.createLEDController();
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

    if (RobotController.getRSLState()) {
      setColor(Color.kOrangeRed);
    } else {
      setColor(Color.kBlack);
    }

    if (mainLightWidget != null) {
      mainLightWidget =
          mainLightWidget.withProperties(Map.of("colorWhenTrue", getColor().toHexString()));
    }
  }

  @Override
  public void addToShuffleboard(ShuffleboardTab tab) {
    ShuffleboardLayout mainColumn = Telemetry.addColumn(tab, "Main Light");

    mainLightWidget =
        mainColumn
            .add("Main Light Color", true)
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
}
