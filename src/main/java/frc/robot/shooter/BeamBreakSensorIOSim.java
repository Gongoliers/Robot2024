package frc.robot.shooter;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Simulated beam break sensor. */
public class BeamBreakSensorIOSim implements BeamBreakSensorIO {

  /** NetworkTables entry for if the beam break sensor is broken. */
  private final BooleanEntry isBrokenEntry;

  /** Creates a simulated beam break sensor. */
  public BeamBreakSensorIOSim() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("shooter/beamBreakSensorSim");

    isBrokenEntry = table.getBooleanTopic("isBroken").getEntry(false);
    isBrokenEntry.set(false);
  }

  @Override
  public void configure() {}

  @Override
  public void update(BeamBreakSensorIOValues values) {
    values.isBroken = isBrokenEntry.get();
  }
}
