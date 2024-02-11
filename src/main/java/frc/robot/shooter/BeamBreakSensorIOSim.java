package frc.robot.shooter;

import edu.wpi.first.networktables.BooleanEntry;
import frc.lib.Telemetry;

/** Simulated beam break sensor. */
public class BeamBreakSensorIOSim implements BeamBreakSensorIO {

  /** NetworkTables entry for if the beam break sensor is broken. */
  private final BooleanEntry isBrokenEntry;

  /** Creates a simulated beam break sensor. */
  public BeamBreakSensorIOSim() {
    isBrokenEntry = Telemetry.addBooleanEntry("shooter/beamBreakSensorSim", "isBroken");
  }

  @Override
  public void configure() {}

  @Override
  public void update(BeamBreakSensorIOValues values) {
    values.isBroken = isBrokenEntry.get();
  }
}
