package frc.robot.arm;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchIODigital implements LimitSwitchIO {

  private final DigitalInput digitalInput;

  public LimitSwitchIODigital() {
    digitalInput = new DigitalInput(4);
  }

  @Override
  public void configure() {}

  @Override
  public void update(LimitSwitchIOValues values) {
    values.isPressed = !digitalInput.get();
  }
}
