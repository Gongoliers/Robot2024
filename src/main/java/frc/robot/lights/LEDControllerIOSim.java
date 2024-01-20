package frc.robot.lights;

/** Simulated LED controller. */
public class LEDControllerIOSim implements LEDControllerIO {

  /** Red quantity. */
  private double red = 0.0;

  /** Green quantity. */
  private double green = 0.0;

  /** Blue quantity. */
  private double blue = 0.0;

  @Override
  public void configure() {}

  @Override
  public void update(LEDControllerIOValues values) {
    values.red = red;
    values.blue = blue;
    values.green = green;
  }

  @Override
  public void setColor(double red, double green, double blue) {
    this.red = red;
    this.green = green;
    this.blue = blue;
  }
}
