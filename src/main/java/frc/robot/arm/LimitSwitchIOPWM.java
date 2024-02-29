package frc.robot.arm;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchIOPWM implements LimitSwitchIO {

  private final DigitalInput digitalInput;

  public LimitSwitchIOPWM() {
    int port = 0;

    digitalInput = new DigitalInput(port);
  }

  @Override
  public void configure() {}

  @Override
  public void update(LimitSwitchIOValues values) {
    values.isPressed = digitalInput.get();
  }
}
