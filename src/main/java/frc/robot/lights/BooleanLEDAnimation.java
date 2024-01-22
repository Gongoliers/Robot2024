package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BooleanSupplier;

/** LED animation that sets color based on a predicate. */
public class BooleanLEDAnimation implements LEDAnimation {

  /** Condition that determiens the state of the animation. */
  private final BooleanSupplier predicate;

  /** Color of the LED when the predicate is true. */
  private final Color colorWhenTrue;

  /** COlor of the LED when the predicate is true. */
  private final Color colorWhenFalse;

  /**
   * Creates an LED animation.
   *
   * @param predicate the condition that determines the state of the animation.
   * @param colorWhenTrue the color of the LED when the predicate is true.
   * @param colorWhenFalse the color of the LED when the predicate is false.
   */
  public BooleanLEDAnimation(BooleanSupplier predicate, Color colorWhenTrue, Color colorWhenFalse) {
    this.predicate = predicate;
    this.colorWhenTrue = colorWhenTrue;
    this.colorWhenFalse = colorWhenFalse;
  }

  /**
   * Creates an LED animation.
   *
   * @param predicate the condition that determines the state of the animation.
   * @param colorWhenTrue the color of the LED when the predicate is true.
   */
  public BooleanLEDAnimation(BooleanSupplier predicate, Color colorWhenTrue) {
    this(predicate, colorWhenTrue, Color.kBlack);
  }

  @Override
  public Color getColor() {
    if (predicate.getAsBoolean()) {
      return colorWhenTrue;
    }

    return colorWhenFalse;
  }
}
