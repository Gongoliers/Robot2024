package frc.robot.lights;

/** LED controller hardware interface. */
public interface LEDControllerIO {

  /** Values for the LED controller hardware interface. */
  public static class LEDControllerIOValues {
    /** Red quantity. */
    public double red = 0.0;

    /** Green quantity. */
    public double green = 0.0;

    /** Blue quantity. */
    public double blue = 0.0;
  }

  /** Configures the LED controller. */
  public void configure();

  /**
   * Updates the LED controller's values.
   *
   * @param values
   */
  public void update(LEDControllerIOValues values);

  /**
   * Sets the color for the LED controller.
   *
   * @param red the quantity of red.
   * @param green the quantity of green.
   * @param blue the quantity of blue.
   */
  public void setColor(double red, double green, double blue);
}
