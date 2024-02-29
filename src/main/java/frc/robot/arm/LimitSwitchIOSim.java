package frc.robot.arm;

import edu.wpi.first.networktables.BooleanEntry;
import frc.lib.Telemetry;

public class LimitSwitchIOSim implements LimitSwitchIO {

  private final BooleanEntry isPressedEntry;

  public LimitSwitchIOSim() {
    isPressedEntry = Telemetry.addBooleanEntry("arm/limitSwitch", "isPressed");
  }

  @Override
  public void configure() {}

  @Override
  public void update(LimitSwitchIOValues values) {
    values.isPressed = isPressedEntry.get();
  }
}
