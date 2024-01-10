package frc.robot.shooter;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;

/** Simulated beam break sensor. */
public class BeamBreakSensorIOSim implements BeamBreakSensorIO {

  /** Supplies values for if the beam break sensor is broken. */
  private final BooleanSupplier isBrokenSupplier;

  /** Creates a simulated beam break sensor. */
  public BeamBreakSensorIOSim() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("shooter/beamBreakSensorSim");

    BooleanTopic isBrokenTopic = table.getBooleanTopic("isBroken");
    isBrokenTopic.publish().set(false);
    isBrokenSupplier = isBrokenTopic.subscribe(false);
  }

  @Override
  public void configure() {}

  @Override
  public void update(BeamBreakSensorIOValues values) {
    values.isBroken = isBrokenSupplier.getAsBoolean();
  }
}
