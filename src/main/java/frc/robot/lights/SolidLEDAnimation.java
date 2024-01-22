package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;

/** LED animation that sets a single color. */
public class SolidLEDAnimation implements LEDAnimation {

  /** Color of the animation. */
  private final Color color;

  /**
   * Creates an LED animation.
   *
   * @param color the color of the animation.
   */
  public SolidLEDAnimation(Color color) {
    this.color = color;
  }

  @Override
  public Color getColor() {
    return color;
  }
}
