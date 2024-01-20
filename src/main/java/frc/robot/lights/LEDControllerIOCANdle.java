package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LEDControllerIOCANdle implements LEDControllerIO {

  /** Hardware CANdle. */
  private final CANdle candle;

  /** Red quantity. */
  private double red = 0.0;

  /** Green quantity. */
  private double green = 0.0;

  /** Blue quantity. */
  private double blue = 0.0;

  public LEDControllerIOCANdle() {
    candle = new CANdle(LightsConstants.CANDLE_ID.id());
  }

  @Override
  public void configure() {
    CANdleConfiguration config = new CANdleConfiguration();

    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;

    candle.configAllSettings(config);
  }

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

    int red255 = (int) (red * 255);
    int green255 = (int) (green * 255);
    int blue255 = (int) (blue * 255);

    candle.setLEDs(red255, green255, blue255);
  }
}
